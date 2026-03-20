package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
        addRequirements(indexer, spindexer, shooter, hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_pose.get().transformBy(new Transform2d(Constants.Turret.kTurretOffset, new Rotation2d()));
        double diffX = Blackbox.getActiveTarget(currentPose).getX() - currentPose.getX();
        double diffY = Blackbox.getActiveTarget(currentPose).getY() - currentPose.getY();
        double distance = Math.hypot(diffX, diffY);
        double flywheelRPM = lerp(Constants.Shooter.distanceTable, distance);
        
        //Flywheel only spin when held
        if (m_shoot.getAsDouble() > 0.3) {
            m_shooter.setFlywheelTargetRPM(flywheelRPM);
        }
        else {
            m_shooter.setFlywheelTargetRPM(0);
            m_indexer.setIndexerRPM(0);
            m_spindexer.setSpindexerRPM(0);
        }

        //Hood Angle based of zone
        if (FieldZoneUtil.getZone(currentPose) == Zone.CROSSING) {
            m_shooterHood.setSetpoint(0);
        }
        else if (FieldZoneUtil.getZone(currentPose) == Zone.NEUTRAL) {
            m_shooterHood.setSetpoint(28);
        }
        else {
            m_shooterHood.setSetpoint(Constants.ShooterHood.kFixedHood);
        }

        //Act start shooting
        if (m_shooter.isFlywheelAtSpeed() && m_shooterHood.atSetpoint() && m_shoot.getAsDouble() > 0.3 && distance > 1.6) {
            m_indexer.setIndexerRPM(Constants.Indexer.m_indexerRPM);
            m_spindexer.setSpindexerRPM(Constants.Spindexer.m_spindexerRPM);
        }
        Logger.recordOutput("Shooter In Range", distance > 1.6);
        Logger.recordOutput("Debug/distance", distance);
        Logger.recordOutput("Debug/turret pose", currentPose);
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
        m_spindexer.stop();
        m_shooter.stop();
        m_shooterHood.setSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static double lerp(double[][] table, double input) {
    // Below minimum — clamp to first entry
    if (input <= table[0][0]) return table[0][1];

    // Above maximum — clamp to last entry
    if (input >= table[table.length - 1][0]) return table[table.length - 1][1];

    // Find the two surrounding entries and interpolate
    for (int i = 0; i < table.length - 1; i++) {
        if (input >= table[i][0] && input < table[i + 1][0]) {
            double t = (input - table[i][0]) / (table[i + 1][0] - table[i][0]);
            return table[i][1] + t * (table[i + 1][1] - table[i][1]);
        }
    }

    return table[table.length - 1][1]; // Fallback
}
}