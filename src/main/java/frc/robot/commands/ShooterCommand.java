package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.vision.Vision;

public class ShooterCommand extends Command {
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    private final Indexer m_indexer;
    private final Spindexer m_spindexer;
    private final Vision m_vision;

    public ShooterCommand(Shooter m_shooter, Indexer m_indexer, Spindexer m_spindexer, ShooterHood m_shooterHood, Vision m_vision) {
        this.m_shooter = m_shooter;
        this.m_shooterHood = m_shooterHood;
        this.m_vision = m_vision;
        this.m_indexer = m_indexer;
        this.m_spindexer = m_spindexer;
        addRequirements(m_shooter, m_indexer, m_spindexer, m_shooterHood);
    }

    @Override
    public void execute() {
        if (m_vision.hasPose()) {
            Pose2d robotPose = m_vision.getPose2d();
            double deltaX = Constants.Vision.kGoalX - robotPose.getX();
            double deltaY = Constants.Vision.kGoalY - robotPose.getY();
            double dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

            if (dist > Constants.Ranges.dist8) {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm8);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm8);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm8);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle8);
            } else if (dist > Constants.Ranges.dist7) {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm7);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm7);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm7);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle7);
            } else if (dist > Constants.Ranges.dist6) {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm6);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm6);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm6);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle6);
            } else if (dist > Constants.Ranges.dist5) {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm5);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm5);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm5);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle5);
            } else if (dist > Constants.Ranges.dist4) {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm4);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm4);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm4);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle4);
            } else if (dist > Constants.Ranges.dist3) {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm3);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm3);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm3);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle3);
            } else if (dist > Constants.Ranges.dist2) {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm2);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm2);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm2);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle2);
            } else {
                m_shooter.setFlywheelTargetRPM(Constants.Shooter.rpm1);
                m_indexer.setIndexerRPM(Constants.Indexer.rpm1);
                m_spindexer.setSpindexerRPM(Constants.Spindexer.rpm1);
                m_shooterHood.setSetpoint(Constants.ShooterHood.angle1);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}