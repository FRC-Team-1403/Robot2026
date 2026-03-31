package team1403.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.Blackbox;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Zone;

public class SOTMVectorSubCommand extends Command {
    private final Turret m_turret;
    private final Indexer m_indexer;
    private final Spindexer m_spindexer;
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    private final Supplier<Pose2d> m_pose;
    private final DoubleSupplier m_shoot;
    private final Supplier<ChassisSpeeds> m_chassisSupplier;
    private boolean isShooting;
    private boolean wasShooting;
    private Timer backupTimer;

    public SOTMVectorSubCommand(
            Supplier<ChassisSpeeds> chassisSupplier,
            Turret turret,
            Indexer indexer,
            Spindexer spindexer,
            Shooter shooter,
            ShooterHood hood,
            Supplier<Pose2d> pose,
            DoubleSupplier shoot) {
        m_chassisSupplier = chassisSupplier;
        m_turret = turret;
        m_indexer = indexer;
        m_spindexer = spindexer;
        m_shooter = shooter;
        m_shooterHood = hood;
        m_pose = pose;
        m_shoot = shoot;
        isShooting = false;
        wasShooting = false;
        backupTimer = new Timer();
        addRequirements(indexer, spindexer, shooter, hood, turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // If shooting just stopped, start backup timer to reverse the feed path
        if (wasShooting && !isShooting) {
            backupTimer.reset();
            backupTimer.start();
        }
        wasShooting = isShooting;

        Pose2d robotPose = m_pose.get();
        // Convert robot-relative chassis speeds to field-relative velocity vector
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                m_chassisSupplier.get(),
                robotPose.getRotation()
        );
        Translation2d robotVelocity = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond
        );
        boolean humanInput = m_shoot.getAsDouble() > 0.3;

        // Turret pivot offset from robot center in field coordinates
        Translation2d turretPivot = robotPose.getTranslation()
                .plus(Constants.Turret.kTurretOffset.rotateBy(robotPose.getRotation()));

        // Project turret pivot forward in time to account for camera + CAN + motor lag
        Translation2d futurePos = turretPivot.plus(
                robotVelocity.times(Constants.Shooter.kLatencyCompensation)
        );

        Translation2d target = Blackbox.getActiveTarget(robotPose);
        Translation2d toGoal = target.minus(futurePos);
        double distance = toGoal.getNorm();

        // Look up baseline horizontal exit velocity using: v_horizontal = distance / ToF
        double timeOfFlight = lerp(Constants.Shooter.kTOFTable, distance);
        double baselineHorizontalVelocity = distance / timeOfFlight;

        // Build the target velocity vector: direction to goal * baseline speed
        // This is what the ball needs to do horizontally to reach the goal if standing still
        Translation2d targetVelocity = toGoal.div(distance).times(baselineHorizontalVelocity);

        // V_shot/robot = V_shot/ground - V_robot/ground
        // We need the ball to travel targetVelocity relative to the field,
        // so relative to the robot it must be offset by our own velocity
        Translation2d shotVector = targetVelocity.minus(robotVelocity);

        double fieldAngleToGoal = shotVector.getAngle().getDegrees();
        double robotHeading = robotPose.getRotation().getDegrees();
        // Convert field angle to turret-relative angle
        double turretAngle = MathUtil.inputModulus(
                fieldAngleToGoal - robotHeading - 90,
                Constants.Turret.kMinAngleDegrees,
                Constants.Turret.kMaxAngleDegrees
        );
        m_turret.setSetpoint(turretAngle);

        // Required horizontal exit velocity after motion compensation
        double requiredHorizontalVelocity = shotVector.getNorm();

        // Since our LUT maps distance → velocity (via ToF), we find the distance
        // that produces the required velocity and use its RPM/hood values.
        // This keeps us anchored to empirically-tuned values, no physics model needed.
        double effectiveDistance = velocityToEffectiveDistance(requiredHorizontalVelocity);
        double flywheelRPM = lerp(Constants.Shooter.distanceTable, effectiveDistance);
        double hoodAngle   = lerp(Constants.ShooterHood.distanceTable, effectiveDistance);

        Zone zone = FieldZoneUtil.getZone(robotPose);
        boolean isInHubShadow = zone.equals(Zone.HUB_SHADOW);

        if (humanInput && !isInHubShadow) {
            m_shooter.setFlywheelTargetRPM(flywheelRPM);

            if (zone == Zone.CROSSING) {
                m_shooterHood.setSetpoint(0);
            } else {
                m_shooterHood.setSetpoint(hoodAngle);
            }
        } else {
            isShooting = false;
            m_shooter.setFlywheelTargetRPM(0);
            m_shooterHood.setSetpoint(0);

            if (backupTimer.isRunning() && backupTimer.get() < Constants.Shooter.kBackupTime) {
                m_spindexer.setSpindexerRPM(-2000);
                m_indexer.setIndexerRPM(-1800);
            } else {
                backupTimer.stop();
                backupTimer.reset();
                m_spindexer.setSpindexerRPM(0);
                m_indexer.setIndexerRPM(0);
            }
        }

        // Fire once flywheel, hood, and turret are all at setpoint
        if (m_shooter.isFlywheelAtSpeed()
                && m_shooterHood.atSetpoint()
                && humanInput
                && !isInHubShadow
                && m_turret.atSetpoint()) {
            isShooting = true;
            m_spindexer.setSpindexerRPM(Constants.Spindexer.m_spindexerRPM);
            m_indexer.setIndexerRPM(Constants.Indexer.m_indexerRPM);
        }

        SmartDashboard.putNumber("Debug/distance", distance);
        Logger.recordOutput("LERPShooter/RobotPose", robotPose);
        Logger.recordOutput("LERPShooter/FuturePos", new Pose2d(futurePos, robotPose.getRotation()));
        Logger.recordOutput("LERPShooter/ActiveTarget", new Pose2d(target, new Rotation2d()));
        Logger.recordOutput("LERPShooter/FieldAngleToGoal", fieldAngleToGoal);
        Logger.recordOutput("LERPShooter/TurretAngle", turretAngle);
        Logger.recordOutput("LERPShooter/HoodAngle", hoodAngle);
        Logger.recordOutput("LERPShooter/Distance", distance);
        Logger.recordOutput("LERPShooter/EffectiveDistance", effectiveDistance);
        Logger.recordOutput("LERPShooter/BaselineHorizVelocity", baselineHorizontalVelocity);
        Logger.recordOutput("LERPShooter/RequiredHorizVelocity", requiredHorizontalVelocity);
        Logger.recordOutput("LERPShooter/FlywheelRPM", flywheelRPM);
        Logger.recordOutput("LERPShooter/IsShooting", isShooting);
    }

    @Override
    public void end(boolean interrupted) {
        m_spindexer.stop();
        m_indexer.stop();
        m_shooter.stop();
        m_shooterHood.setSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // velocity = distance/tof 
    // Takes y
    private double velocityToEffectiveDistance(double requiredVelocity) {
        double[][] tof = Constants.Shooter.kTOFTable;

        // Clamp to table bounds
        double minVel = tof[0][0] / tof[0][1];             // distance / ToF at near end
        double maxVel = tof[tof.length - 1][0] / tof[tof.length - 1][1]; // far end

        if (requiredVelocity <= minVel) return tof[0][0];
        if (requiredVelocity >= maxVel) return tof[tof.length - 1][0];

        // Walk the table and interpolate between the two entries that bracket the velocity
        for (int i = 0; i < tof.length - 1; i++) {
            double vel0 = tof[i][0]     / tof[i][1];
            double vel1 = tof[i + 1][0] / tof[i + 1][1];

            if (requiredVelocity >= vel0 && requiredVelocity < vel1) {
                double t = (requiredVelocity - vel0) / (vel1 - vel0);
                return tof[i][0] + t * (tof[i + 1][0] - tof[i][0]);
            }
        }

        return tof[tof.length - 1][0];
    }

    //lerp table for shooter rpm and hood
    public static double lerp(double[][] table, double input) {
        if (input <= table[0][0]) return table[0][1];
        if (input >= table[table.length - 1][0]) return table[table.length - 1][1];

        for (int i = 0; i < table.length - 1; i++) {
            if (input >= table[i][0] && input < table[i + 1][0]) {
                double t = (input - table[i][0]) / (table[i + 1][0] - table[i][0]);
                return table[i][1] + t * (table[i + 1][1] - table[i][1]);
            }
        }

        return table[table.length - 1][1];
    }
}