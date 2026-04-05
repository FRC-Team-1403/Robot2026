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
    private Pose2d m_lastProjectedPivot;

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
        m_lastProjectedPivot = new Pose2d();
        addRequirements(indexer, spindexer, shooter, hood, turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //Get updated values
        Pose2d robotPose = m_pose.get();
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(m_chassisSupplier.get(), robotPose.getRotation());
        boolean humanInput = m_shoot.getAsDouble() > 0.3;

        //Transform robot into turret
        Translation2d turretPivotField = robotPose.getTranslation()
                .plus(Constants.Turret.kTurretOffset.rotateBy(robotPose.getRotation()));

        //Calculate current values
        Translation2d target = Blackbox.getActiveTarget(robotPose);
        double deltaX = target.getX() - turretPivotField.getX();
        double deltaY = target.getY() - turretPivotField.getY();
        double distance = Math.hypot(deltaX, deltaY);

        //Calculate future values
        double shotTime = lerp(Constants.Shooter.kTOFTable, distance);
        shotTime += Constants.Shooter.latency;
        double offsetX = shotTime * robotVelocity.vxMetersPerSecond;
        double offsetY = shotTime * robotVelocity.vyMetersPerSecond;
        Translation2d turret = Constants.Turret.kTurretOffset.rotateBy(robotPose.getRotation());
        double turretX = -1 * shotTime * robotVelocity.omegaRadiansPerSecond * turret.getY();
        double turretY =  1 * shotTime * robotVelocity.omegaRadiansPerSecond * turret.getX();
        Pose2d projectedPivot = new Pose2d(
                turretPivotField.getX() + offsetX + turretX,
                turretPivotField.getY() + offsetY + turretY,
                robotPose.getRotation());

        //Recompute distance
        double projDeltaX = target.getX() - projectedPivot.getX();
        double projDeltaY = target.getY() - projectedPivot.getY();
        double projectedDistance = Math.hypot(projDeltaX, projDeltaY);

        //Find and set turret
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(projDeltaY, projDeltaX));
        double robotHeading = projectedPivot.getRotation().getDegrees();
        double turretAngle = MathUtil.inputModulus(
                fieldAngleToGoal - robotHeading - Constants.Turret.rotationCorrectionOffset,
                Constants.Turret.kMinAngleDegrees,
                Constants.Turret.kMaxAngleDegrees);
        m_turret.setSetpoint(turretAngle);

        //LERP flywheel and hood speed/angle
        double flywheelRPM = lerp(Constants.Shooter.distanceTable, projectedDistance);
        double hoodAngle = lerp(Constants.ShooterHood.distanceTable, projectedDistance);

        //If shoot button is pressed
        if (humanInput) {
            //Set flywheel
            m_shooter.setFlywheelTargetRPM(flywheelRPM);

            //Set hood
            Zone zone = FieldZoneUtil.getZone(robotPose);
            if (zone == Zone.CROSSING) {
                m_shooterHood.setSetpoint(0);
            }
            else {
                m_shooterHood.setSetpoint(hoodAngle);
            }
        } 
        //Otherwise reset to safe conditions
        else {
            m_shooter.setFlywheelTargetRPM(0);
            m_shooterHood.setSetpoint(0);

            //If currently backing up
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

        //Calculate shot stability
        double translationDelta = m_lastProjectedPivot.getTranslation().getDistance(projectedPivot.getTranslation());
        double rotationDelta = Math.abs(MathUtil.angleModulus(projectedPivot.getRotation().getRadians() - m_lastProjectedPivot.getRotation().getRadians()));
        double stabilityScore = 100 * translationDelta + 500 * rotationDelta;

        boolean allowedToShoot = m_shooter.isFlywheelAtSpeed()
                                && m_shooterHood.atSetpoint()
                                && humanInput
                                && m_turret.atSetpoint()
                                && stabilityScore < 6; //Tuned

        //Should we actually start shooting
        if (allowedToShoot) {
            isShooting = true;
            m_spindexer.setSpindexerRPM(Constants.Spindexer.m_spindexerRPM);
            m_indexer.setIndexerRPM(Constants.Indexer.m_indexerRPM);
        }
        else {
            isShooting = false;
        }

        //If shooting changed from true to false
        if (wasShooting && !isShooting) {
            backupTimer.reset();
            backupTimer.start();
        }

        //Update values
        wasShooting = isShooting;
        m_lastProjectedPivot = projectedPivot;

        //Logging
        SmartDashboard.putNumber("Debug/distance", distance);
        Logger.recordOutput("LERPShooter/RobotPose", robotPose);
        Logger.recordOutput("LERPShooter/ProjectedPivot", projectedPivot);
        Logger.recordOutput("LERPShooter/ActiveTarget", new Pose2d(target, new Rotation2d()));
        Logger.recordOutput("LERPShooter/FieldAngleToGoal", fieldAngleToGoal);
        Logger.recordOutput("LERPShooter/HoodAngle", hoodAngle);
        Logger.recordOutput("LERPShooter/TurretAngle", turretAngle);
        Logger.recordOutput("LERPShooter/ProjectedDistance", projectedDistance);
        Logger.recordOutput("LERPShooter/FlywheelRPM", flywheelRPM);
        Logger.recordOutput("LERPShooter/IsShooting", isShooting);
        Logger.recordOutput("LERPShooter", allowedToShoot);
        Logger.recordOutput("LERPShooter/Stability", stabilityScore);
        Logger.recordOutput("LERPShooter/Turret Posistion", new Pose2d(turretPivotField.getX(), turretPivotField.getY(), new Rotation2d(turretAngle*Math.PI/180).plus(robotPose.getRotation()))
);
    }

    //Should never run (Default Command)
    @Override
    public void end(boolean interrupted) {
        m_spindexer.stop();
        m_indexer.stop();
        m_shooter.stop();
        m_shooterHood.setSetpoint(0);
    }

    //Should never run (Default Command)
    @Override
    public boolean isFinished() {
        return false;
    }

    //LERP helper method
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