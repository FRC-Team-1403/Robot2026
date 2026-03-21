package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.util.Blackbox;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Zone;

public class LERPShooter extends Command {
    private final Indexer m_indexer;
    private final Spindexer m_spindexer;
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    private final Supplier<Pose2d> m_pose;
    private final DoubleSupplier m_shoot;
    private boolean isShooting;
    private Timer backupTimer;
    private boolean wasShooting;

    public LERPShooter(
            Indexer indexer,
            Spindexer spindexer,
            Shooter shooter,
            ShooterHood hood,
            Supplier<Pose2d> pose,
            DoubleSupplier shoot) {
        m_indexer = indexer;
        m_spindexer = spindexer;
        m_shooter = shooter;
        m_shooterHood = hood;
        m_pose = pose;
        m_shoot = shoot;
        isShooting = false;
        wasShooting = false;
        backupTimer = new Timer();
        addRequirements(indexer, spindexer, shooter, hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (wasShooting && !isShooting) {
            backupTimer.reset();
            backupTimer.start();
        }
        wasShooting = isShooting;

        Pose2d currentPose = m_pose.get().transformBy(new Transform2d(Constants.Turret.kTurretOffset, new Rotation2d()));
        double diffX = Blackbox.getActiveTarget(currentPose).getX() - currentPose.getX();
        double diffY = Blackbox.getActiveTarget(currentPose).getY() - currentPose.getY();
        double distance = Math.hypot(diffX, diffY);
        double flywheelRPM = lerp(Constants.Shooter.distanceTable, distance);

        if (m_shoot.getAsDouble() > 0.3) {
            m_shooter.setFlywheelTargetRPM(flywheelRPM);

            if (FieldZoneUtil.getZone(currentPose) == Zone.CROSSING) {
                m_shooterHood.setSetpoint(0);
            } else if (FieldZoneUtil.getZone(currentPose) == Zone.NEUTRAL) {
                m_shooterHood.setSetpoint(28);
            } else {
                m_shooterHood.setSetpoint(Constants.ShooterHood.kFixedHood);
            }
        } else {
            isShooting = false;
            m_shooter.setFlywheelTargetRPM(0);
            m_shooterHood.setSetpoint(0);

            if (backupTimer.isRunning() && backupTimer.get() < 0.2) {
                m_spindexer.setSpindexerRPM(-2000);
                m_indexer.setIndexerRPM(-1800);
            } else {
                backupTimer.stop();
                backupTimer.reset();
                m_spindexer.setSpindexerRPM(0);
                m_indexer.setIndexerRPM(0);
            }
        }

        if (m_shooter.isFlywheelAtSpeed() && m_shooterHood.atSetpoint() && m_shoot.getAsDouble() > 0.3 && distance > 1.6) {
            isShooting = true;
            m_spindexer.setSpindexerRPM(Constants.Spindexer.m_spindexerRPM);
            m_indexer.setIndexerRPM(Constants.Indexer.m_indexerRPM);
        }

        SmartDashboard.putBoolean("Debug/Shooter In Range", distance > 1.6);
        SmartDashboard.putNumber("Debug/distance", distance);
        //SmartDashboard.putNumber("Debug/turret pose", currentPose);
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