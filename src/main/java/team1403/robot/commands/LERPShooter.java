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

public class LERPShooter extends Command {
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

    public LERPShooter(
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
        // If shooting just stopped, start backup timer
        if (wasShooting && !isShooting) {
            backupTimer.reset();
            backupTimer.start();
        }

        wasShooting = isShooting;

        // Get updated robot state
        Pose2d robotPose = m_pose.get();
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                m_chassisSupplier.get(),
                robotPose.getRotation()
        );
        boolean humanInput = m_shoot.getAsDouble() > 0.3;

        // Turret pivot in field coordinates
        Translation2d turretPivotField = robotPose.getTranslation()
                .plus(Constants.Turret.kTurretOffset.rotateBy(robotPose.getRotation()));

        // Get current target
        Translation2d target = Blackbox.getActiveTarget(robotPose);

        // ---- Iterative future prediction with rotation compensation ----
        final int kIterations = 3; // Number of refinements
        Pose2d projectedPivot = new Pose2d(turretPivotField.getX(), turretPivotField.getY(), robotPose.getRotation());
        double projectedDistance = 0;

        // Angular velocity of the robot in radians per second
        double omega = robotVelocity.omegaRadiansPerSecond;

        for (int i = 0; i < kIterations; i++) {
            double deltaX = target.getX() - projectedPivot.getX();
            double deltaY = target.getY() - projectedPivot.getY();
            projectedDistance = Math.hypot(deltaX, deltaY);

            double shotTime = lerp(Constants.Shooter.kTOFTable, projectedDistance) + Constants.Shooter.kLatencyCompensation;

            // Predict translational offset
            double offsetX = shotTime * robotVelocity.vxMetersPerSecond;
            double offsetY = shotTime * robotVelocity.vyMetersPerSecond;

            // Predict rotational offset
            Rotation2d rotationOffset = new Rotation2d(omega * shotTime);

            // Apply rotation to turret pivot offset
            Translation2d rotatedOffset = new Translation2d(offsetX, offsetY).rotateBy(rotationOffset);

            // Update projected pivot
            projectedPivot = new Pose2d(
                    turretPivotField.getX() + rotatedOffset.getX(),
                    turretPivotField.getY() + rotatedOffset.getY(),
                    robotPose.getRotation().plus(rotationOffset)
            );
        }

        // Turret angle calculation
        double projDeltaX = target.getX() - projectedPivot.getX();
        double projDeltaY = target.getY() - projectedPivot.getY();
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(projDeltaY, projDeltaX));
        double robotHeading = projectedPivot.getRotation().getDegrees();
        double turretAngle = MathUtil.inputModulus(
                fieldAngleToGoal - robotHeading - 90,
                Constants.Turret.kMinAngleDegrees,
                Constants.Turret.kMaxAngleDegrees
        );
        m_turret.setSetpoint(turretAngle);

        // Flywheel and hood LERP
        double flywheelRPM = lerp(Constants.Shooter.distanceTable, projectedDistance);
        double hoodAngle = lerp(Constants.ShooterHood.distanceTable, projectedDistance);

        Zone zone = FieldZoneUtil.getZone(robotPose);
        boolean isInHubShadow = zone.equals(Zone.HUB_SHADOW);

        // Shooting logic
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

        // Start shooting if all conditions met
        if (m_shooter.isFlywheelAtSpeed()
                && m_shooterHood.atSetpoint()
                && humanInput
                && !isInHubShadow
                && m_turret.atSetpoint()) {
            isShooting = true;
            m_spindexer.setSpindexerRPM(Constants.Spindexer.m_spindexerRPM);
            m_indexer.setIndexerRPM(Constants.Indexer.m_indexerRPM);
        }

        // Logging
        SmartDashboard.putNumber("Debug/distance", projectedDistance);
        Logger.recordOutput("LERPShooter/RobotPose", robotPose);
        Logger.recordOutput("LERPShooter/ProjectedPivot", projectedPivot);
        Logger.recordOutput("LERPShooter/ActiveTarget", new Pose2d(target, new Rotation2d()));
        Logger.recordOutput("LERPShooter/FieldAngleToGoal", fieldAngleToGoal);
        Logger.recordOutput("LERPShooter/HoodAngle", hoodAngle);
        Logger.recordOutput("LERPShooter/TurretAngle", turretAngle);
        Logger.recordOutput("LERPShooter/ProjectedDistance", projectedDistance);
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

    // LERP helper
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
