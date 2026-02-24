package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import org.littletonrobotics.junction.Logger;

public class TurretCommand extends Command {

    // -------------------------------------------------------------------------
    // Ball physical constants — tune these empirically from real shot data
    // -------------------------------------------------------------------------
    private static final double BALL_MASS         = 0.27;    // kg
    private static final double BALL_RADIUS       = 0.127;   // m (5" ball)
    private static final double BALL_AREA         = Math.PI * BALL_RADIUS * BALL_RADIUS;
    private static final double CD                = 0.55;    // drag coeff, tune from data
    private static final double AIR_DENSITY       = 1.225;   // kg/m³, adjust for event altitude
    private static final double MAGNUS_CL         = 0.40;    // lift coeff, tune from data
    private static final double MAGNUS_S          = 0.5 * MAGNUS_CL * AIR_DENSITY * BALL_AREA * BALL_RADIUS;
    private static final double DRAG_K            = 0.5 * CD * AIR_DENSITY * BALL_AREA / BALL_MASS;

    // Flywheel → ball exit velocity
    private static final double WHEEL_DIAMETER_M  = 0.1016;  // m (4" wheel), change if different
    private static final double EXIT_EFFICIENCY   = 0.85;    // tune from real shot data

    // Hood wheel gear ratio relative to main flywheel.
    // e.g. 1.5 means hood spins at flywheelRPM / 1.5
    // Hood wheels spin OPPOSITE to main flywheel → backspin on the ball
    // Backspin → Magnus lifts ball UP, extending range
    // Defined in Constants.Turret.kHoodGearRatio

    // Spin transfer fraction — how much of wheel surface speed becomes ball spin
    private static final double SPIN_TRANSFER     = 0.70;    // tune empirically

    // Solver resolution
    private static final double HOOD_STEP_DEG     = 1.0;
    private static final double RPM_STEP          = 150.0;
    private static final double SIM_DT            = 0.005;   // 5ms RK4 timestep
    private static final double MAX_FLIGHT_TIME   = 4.0;     // s

    // Motion compensation iterations (converges in ~3-4)
    private static final int    LEAD_ITERATIONS   = 5;

    // -------------------------------------------------------------------------

    private final Turret     m_turret;
    private final Drivetrain m_drivetrain;

    // Reusable arrays — avoids per-loop heap allocation on the RIO
    private final double[] m_k1  = new double[6];
    private final double[] m_k2  = new double[6];
    private final double[] m_k3  = new double[6];
    private final double[] m_k4  = new double[6];

    // Ball spin vector, set once per candidate solve, read inside deriv()
    private double m_spinX, m_spinY, m_spinZ;

    public TurretCommand(Turret turret, Drivetrain drivetrain) {
        this.m_turret     = turret;
        this.m_drivetrain = drivetrain;
        addRequirements(m_turret);
    }

    // =========================================================================
    // Command loop
    // =========================================================================

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getPose(); // fused odometry + vision

        ChassisSpeeds speeds = m_drivetrain.getChassisSpeeds(); // field-relative

        double shooterX = pose.getX();
        double shooterY = pose.getY();
        double shooterZ = Constants.Turret.kShooterHeightM;

        double targetX  = Constants.Vision.kGoalX;
        double targetY  = Constants.Vision.kGoalY;
        double targetZ  = Constants.Vision.kGoalHeightM;

        double robotHeadingRad = pose.getRotation().getRadians();

        // ----- Iterative motion compensation ---------------------------------
        // Guess flight time, predict where robot will be, solve ballistics,
        // refine flight time from solution, repeat until convergence
        double flightTime = 0.5; // initial guess in seconds
        ShootingParams params = null;

        for (int i = 0; i < LEAD_ITERATIONS; i++) {
            // Virtual shooter position when ball arrives
            double virtX = shooterX + speeds.vxMetersPerSecond * flightTime;
            double virtY = shooterY + speeds.vyMetersPerSecond * flightTime;

            double dx        = targetX - virtX;
            double dy        = targetY - virtY;
            double horizDist = Math.hypot(dx, dy);

            params = solveWithPhysics(virtX, virtY, shooterZ, targetX, targetY, targetZ);
            if (params == null) break;

            // Refine flight time using horizontal component of exit velocity
            double exitV  = rpmToVelocity(params.rpm);
            double vHoriz = exitV * Math.cos(Math.toRadians(params.hoodAngle));
            if (vHoriz > 0.1) flightTime = horizDist / vHoriz;
        }

        if (params == null) {
            Logger.recordOutput("TurretCommand/SolverStatus", "NO_SOLUTION");
            return;
        }

        // Final virtual position for turret angle calculation
        double virtX = shooterX + speeds.vxMetersPerSecond * flightTime;
        double virtY = shooterY + speeds.vyMetersPerSecond * flightTime;

        // ----- Turret angle --------------------------------------------------
        double fieldAngle  = Math.toDegrees(Math.atan2(targetY - virtY, targetX - virtX));
        double turretAngle = MathUtil.inputModulus(
                fieldAngle - Math.toDegrees(robotHeadingRad), -180.0, 180.0);
        turretAngle = MathUtil.clamp(
                turretAngle, Constants.Turret.kMinAngleDeg, Constants.Turret.kMaxAngleDeg);

        // ----- Apply setpoints -----------------------------------------------
        m_turret.setSetpoint(turretAngle);
        m_turret.setHoodAngle(params.hoodAngle);
        m_turret.setFlywheelRPM(params.rpm);

        // ----- Logging -------------------------------------------------------
        Logger.recordOutput("TurretCommand/RobotPose",         pose);
        Logger.recordOutput("TurretCommand/GoalPose",
                new Pose2d(targetX, targetY, new Rotation2d()));
        Logger.recordOutput("TurretCommand/VirtX",             virtX);
        Logger.recordOutput("TurretCommand/VirtY",             virtY);
        Logger.recordOutput("TurretCommand/FlightTimeSec",     flightTime);
        Logger.recordOutput("TurretCommand/TurretAngleDeg",    turretAngle);
        Logger.recordOutput("TurretCommand/HoodAngleDeg",      params.hoodAngle);
        Logger.recordOutput("TurretCommand/FlywheelRPM",       params.rpm);
        Logger.recordOutput("TurretCommand/SolverLandingError",params.landingError);
        Logger.recordOutput("TurretCommand/RobotVX",           speeds.vxMetersPerSecond);
        Logger.recordOutput("TurretCommand/RobotVY",           speeds.vyMetersPerSecond);
        Logger.recordOutput("TurretCommand/SolverStatus",      "OK");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // =========================================================================
    // Physics solver
    // =========================================================================

    /**
     * Iterates over all valid hood angle + RPM combinations, simulates the full
     * RK4 trajectory for each, and returns the params that land closest to the
     * target. Returns null if no combination reaches the target.
     */
    private ShootingParams solveWithPhysics(
            double virtX,   double virtY,   double shooterZ,
            double targetX, double targetY, double targetZ) {

        double bestError = Double.MAX_VALUE;
        double bestHood  = Double.NaN;
        double bestRPM   = Double.NaN;

        // Bearing from virtual shooter toward target
        double bearing = Math.atan2(targetY - virtY, targetX - virtX);
        double cosBear = Math.cos(bearing);
        double sinBear = Math.sin(bearing);

        for (double hoodDeg = Constants.Turret.kMinHoodDeg;
             hoodDeg <= Constants.Turret.kMaxHoodDeg;
             hoodDeg += HOOD_STEP_DEG) {

            double hoodRad = Math.toRadians(hoodDeg);
            double cosHood = Math.cos(hoodRad);
            double sinHood = Math.sin(hoodRad);

            for (double rpm = Constants.Turret.kMinRPM;
                 rpm <= Constants.Turret.kMaxRPM;
                 rpm += RPM_STEP) {

                double exitV = rpmToVelocity(rpm);

                // Decompose exit velocity into field-relative components
                double vh = exitV * cosHood;
                double vx = vh * cosBear;
                double vy = vh * sinBear;
                double vz = exitV * sinHood;

                // ----- Spin vector -------------------------------------------
                // Main flywheel and hood wheels spin opposite directions.
                // Net effect on ball: BACKSPIN (spin axis points to the left of
                // the shot direction when viewed from above).
                // Backspin → Magnus force pushes ball UPWARD, extending range.
                //
                // Main flywheel surface speed:
                double mainSurface = rpmToSurfaceSpeed(rpm);
                // Hood wheel surface speed (opposite direction, so subtract):
                double hoodSurface = rpmToSurfaceSpeed(rpm / Constants.Turret.kHoodGearRatio);
                // Net surface speed at ball contact → net spin
                double netSurface  = mainSurface - hoodSurface;
                double spinMag     = (netSurface * SPIN_TRANSFER) / BALL_RADIUS;

                // Spin axis perpendicular to shot bearing, horizontal.
                // For backspin relative to travel direction:
                // axis = (-sinBear, cosBear, 0) rotated so Magnus pushes up.
                // Sign: if main wheel drives ball forward from below → backspin
                // axis points LEFT of travel = (-sinBear, cosBear, 0)
                m_spinX =  sinBear * spinMag;  // flip sign vs topspin
                m_spinY = -cosBear * spinMag;
                m_spinZ =  0.0;

                // Run RK4 simulation
                SimResult result = simulate(
                        virtX, virtY, shooterZ,
                        vx, vy, vz,
                        targetZ);

                // Score by how close we land to the target (weight height heavily)
                double horizErr  = Math.hypot(result.x - targetX, result.y - targetY);
                double heightErr = Math.abs(result.z - targetZ);
                double totalErr  = horizErr + heightErr * 2.0;

                if (totalErr < bestError) {
                    bestError = totalErr;
                    bestHood  = hoodDeg;
                    bestRPM   = rpm;
                }
            }
        }

        if (Double.isNaN(bestHood)) return null;

        return new ShootingParams(
                MathUtil.clamp(bestHood, Constants.Turret.kMinHoodDeg, Constants.Turret.kMaxHoodDeg),
                MathUtil.clamp(bestRPM,  Constants.Turret.kMinRPM,     Constants.Turret.kMaxRPM),
                bestError);
    }

    // =========================================================================
    // RK4 trajectory simulation
    // =========================================================================

    /**
     * Simulates the ball from initial state until it crosses targetZ (falling)
     * or MAX_FLIGHT_TIME is exceeded. Uses pre-allocated k arrays.
     */
    private SimResult simulate(
            double x0, double y0, double z0,
            double vx0, double vy0, double vz0,
            double targetZ) {

        double x = x0, y = y0, z = z0;
        double vx = vx0, vy = vy0, vz = vz0;
        double t = 0.0;

        while (t < MAX_FLIGHT_TIME) {
            deriv(x,                    y,                    z,
                  vx,                   vy,                   vz,                   m_k1);

            deriv(x  + 0.5*SIM_DT*m_k1[0], y  + 0.5*SIM_DT*m_k1[1], z  + 0.5*SIM_DT*m_k1[2],
                  vx + 0.5*SIM_DT*m_k1[3], vy + 0.5*SIM_DT*m_k1[4], vz + 0.5*SIM_DT*m_k1[5], m_k2);

            deriv(x  + 0.5*SIM_DT*m_k2[0], y  + 0.5*SIM_DT*m_k2[1], z  + 0.5*SIM_DT*m_k2[2],
                  vx + 0.5*SIM_DT*m_k2[3], vy + 0.5*SIM_DT*m_k2[4], vz + 0.5*SIM_DT*m_k2[5], m_k3);

            deriv(x  + SIM_DT*m_k3[0], y  + SIM_DT*m_k3[1], z  + SIM_DT*m_k3[2],
                  vx + SIM_DT*m_k3[3], vy + SIM_DT*m_k3[4], vz + SIM_DT*m_k3[5],               m_k4);

            x  += (SIM_DT / 6.0) * (m_k1[0] + 2*m_k2[0] + 2*m_k3[0] + m_k4[0]);
            y  += (SIM_DT / 6.0) * (m_k1[1] + 2*m_k2[1] + 2*m_k3[1] + m_k4[1]);
            z  += (SIM_DT / 6.0) * (m_k1[2] + 2*m_k2[2] + 2*m_k3[2] + m_k4[2]);
            vx += (SIM_DT / 6.0) * (m_k1[3] + 2*m_k2[3] + 2*m_k3[3] + m_k4[3]);
            vy += (SIM_DT / 6.0) * (m_k1[4] + 2*m_k2[4] + 2*m_k3[4] + m_k4[4]);
            vz += (SIM_DT / 6.0) * (m_k1[5] + 2*m_k2[5] + 2*m_k3[5] + m_k4[5]);

            t += SIM_DT;

            // Stop once ball has crossed target height while descending
            if (z <= targetZ && vz < 0.0) break;
            // Also bail if we've fallen well past target — clearly overshot
            if (z < targetZ - 1.0) break;
        }

        return new SimResult(x, y, z, t);
    }

    /**
     * Equations of motion. Writes [dx,dy,dz,dvx,dvy,dvz] into out[].
     * Forces: gravity + aerodynamic drag + Magnus lift.
     */
    private void deriv(double x,  double y,  double z,
                       double vx, double vy, double vz,
                       double[] out) {
        double speed = Math.sqrt(vx*vx + vy*vy + vz*vz);

        // Drag (already per-unit-mass via DRAG_K)
        double drag = DRAG_K * speed;
        double fdx  = -drag * vx;
        double fdy  = -drag * vy;
        double fdz  = -drag * vz;

        // Magnus: (MAGNUS_S / BALL_MASS) * (omega × v)
        double ms  = MAGNUS_S / BALL_MASS;
        double fmx = ms * (m_spinY * vz - m_spinZ * vy);
        double fmy = ms * (m_spinZ * vx - m_spinX * vz);
        double fmz = ms * (m_spinX * vy - m_spinY * vx);

        out[0] = vx;
        out[1] = vy;
        out[2] = vz;
        out[3] = fdx + fmx;
        out[4] = fdy + fmy;
        out[5] = -9.81 + fdz + fmz;
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /** Flywheel RPM → ball exit velocity (m/s) */
    private static double rpmToVelocity(double rpm) {
        return (rpm / 60.0) * (Math.PI * WHEEL_DIAMETER_M) * EXIT_EFFICIENCY;
    }

    /** RPM → wheel surface speed (m/s) — used for spin calculation */
    private static double rpmToSurfaceSpeed(double rpm) {
        return (rpm / 60.0) * (Math.PI * WHEEL_DIAMETER_M);
    }

    // =========================================================================
    // Data classes
    // =========================================================================

    private static final class ShootingParams {
        final double hoodAngle;    // degrees
        final double rpm;          // main flywheel RPM
        final double landingError; // meters, logged for diagnostics

        ShootingParams(double hoodAngle, double rpm, double landingError) {
            this.hoodAngle    = hoodAngle;
            this.rpm          = rpm;
            this.landingError = landingError;
        }
    }

    private static final class SimResult {
        final double x, y, z, time;

        SimResult(double x, double y, double z, double time) {
            this.x    = x;
            this.y    = y;
            this.z    = z;
            this.time = time;
        }
    }
}