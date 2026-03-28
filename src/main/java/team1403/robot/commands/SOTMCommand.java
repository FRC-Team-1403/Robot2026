package team1403.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.util.Blackbox;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Zone;

public class SOTMCommand extends Command {

    private static final int    MAX_ITERATIONS         = 20;
    private static final double TOF_CONVERGENCE        = 0.01;
    private static final double MIN_SHOOT_DISTANCE     = 1.6;
    private static final double SHOOT_TRIGGER_DEADBAND = 0.3;
    private static final double BACKUP_DURATION        = 0.2;
    private static final double LATENCY                = 0.01;
    private static final double SHOOTER_DELAY          = 0.08;

    private final Turret          m_turret;
    private final Indexer         m_indexer;
    private final Spindexer       m_spindexer;
    private final Shooter         m_shooter;
    private final ShooterHood     m_shooterHood;
    private final SwerveSubsystem m_swerve;

    private final DoubleSupplier m_shoot;

    private boolean isShooting;
    private boolean wasShooting;
    private final Timer backupTimer;

    public SOTMCommand(Turret turret,
                       Indexer indexer,
                       Spindexer spindexer,
                       Shooter shooter,
                       ShooterHood shooterHood,
                       SwerveSubsystem swerve,
                       DoubleSupplier shoot) {
        m_turret      = turret;
        m_indexer     = indexer;
        m_spindexer   = spindexer;
        m_shooter     = shooter;
        m_shooterHood = shooterHood;
        m_swerve      = swerve;
        m_shoot       = shoot;

        isShooting  = false;
        wasShooting = false;
        backupTimer = new Timer();

        addRequirements(turret, indexer, spindexer, shooter, shooterHood);
    }

    @Override
    public void initialize() {
        isShooting  = false;
        wasShooting = false;
        backupTimer.stop();
        backupTimer.reset();
    }

    @Override
    public void execute() {
        if (wasShooting && !isShooting) {
            backupTimer.reset();
            backupTimer.start();
        }
        wasShooting = isShooting;

        Pose2d currentPose = m_swerve.getPose().transformBy(
            new Transform2d(Constants.Turret.kTurretOffset, new Rotation2d()));

        ChassisSpeeds robotVelocity = m_swerve.getState().Speeds;

        Translation2d latencyAdjustment = new Translation2d(
            -LATENCY * robotVelocity.vxMetersPerSecond,
            -LATENCY * robotVelocity.vyMetersPerSecond);
        Rotation2d latencyRotation = new Rotation2d(
            -LATENCY * robotVelocity.omegaRadiansPerSecond);
        currentPose = currentPose.plus(
            new Transform2d(latencyAdjustment, latencyRotation));

        double turretAngle = 0;
        double distance = Math.hypot(
            Blackbox.getActiveTarget(currentPose).getX() - currentPose.getX(),
            Blackbox.getActiveTarget(currentPose).getY() - currentPose.getY());

        double currentTOF = LERPShooter.lerp(Constants.Shooter.kTOFTable, distance);
        double lastTOF;
        int iteration = 0;

        do {
            lastTOF = currentTOF;

            double totalTime = currentTOF + LATENCY + SHOOTER_DELAY;

            Translation2d positionAdjustment = new Translation2d(
                totalTime * robotVelocity.vxMetersPerSecond,
                totalTime * robotVelocity.vyMetersPerSecond);
            Rotation2d rotationAdjustment = new Rotation2d(
                totalTime * robotVelocity.omegaRadiansPerSecond);
            Pose2d projectedPose = currentPose.plus(
                new Transform2d(positionAdjustment, rotationAdjustment));

            double diffX = Blackbox.getActiveTarget(projectedPose).getX() - projectedPose.getX();
            double diffY = Blackbox.getActiveTarget(projectedPose).getY() - projectedPose.getY();

            turretAngle = Math.atan2(diffY, diffX);
            distance    = Math.hypot(diffX, diffY);
            currentTOF  = LERPShooter.lerp(Constants.Shooter.kTOFTable, distance);
            iteration++;

        } while ((Math.abs(currentTOF - lastTOF) > TOF_CONVERGENCE) && iteration < MAX_ITERATIONS);

        double turretAngleRelative = MathUtil.angleModulus(
            turretAngle - currentPose.getRotation().getRadians());
        m_turret.setSetpoint(turretAngleRelative * 180 / Math.PI);

        double flywheelRPM = LERPShooter.lerp(Constants.Shooter.distanceTable, distance);

        if (m_shoot.getAsDouble() > SHOOT_TRIGGER_DEADBAND) {
            m_shooter.setFlywheelTargetRPM(flywheelRPM);

            Zone zone = FieldZoneUtil.getZone(currentPose);
            if (zone == Zone.CROSSING) {
                m_shooterHood.setSetpoint(0);
            } else if (zone == Zone.NEUTRAL) {
                m_shooterHood.setSetpoint(28);
            } else {
                m_shooterHood.setSetpoint(Constants.ShooterHood.kFixedHood);
            }
        } else {
            isShooting = false;
            m_shooter.setFlywheelTargetRPM(0);
            m_shooterHood.setSetpoint(0);

            if (backupTimer.isRunning() && backupTimer.get() < BACKUP_DURATION) {
                m_spindexer.setSpindexerRPM(-2000);
                m_indexer.setIndexerRPM(-1800);
            } else {
                backupTimer.stop();
                backupTimer.reset();
                m_spindexer.setSpindexerRPM(0);
                m_indexer.setIndexerRPM(0);
            }
        }

        if (m_shooter.isFlywheelAtSpeed()
                && m_shooterHood.atSetpoint()
                && m_turret.atSetpoint()
                && m_shoot.getAsDouble() > SHOOT_TRIGGER_DEADBAND
                && distance > MIN_SHOOT_DISTANCE) {
            isShooting = true;
            m_spindexer.setSpindexerRPM(Constants.Spindexer.m_spindexerRPM);
            m_indexer.setIndexerRPM(Constants.Indexer.m_indexerRPM);
        }

        SmartDashboard.putBoolean("Debug/SOTM In Range",     distance > MIN_SHOOT_DISTANCE);
        SmartDashboard.putNumber ("Debug/SOTM Distance",     distance);
        SmartDashboard.putNumber ("Debug/SOTM TOF",          currentTOF);
        SmartDashboard.putNumber ("Debug/SOTM Turret Angle", Math.toDegrees(turretAngleRelative));
        SmartDashboard.putNumber ("Debug/SOTM Iterations",   iteration);
        SmartDashboard.putBoolean("Debug/SOTM Is Shooting",  isShooting);
    }

    @Override
    public void end(boolean interrupted) {
        m_spindexer.setSpindexerRPM(0);
        m_indexer.setIndexerRPM(0);
        m_shooter.stop();
        m_shooterHood.setSetpoint(0);
        backupTimer.stop();
        backupTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}