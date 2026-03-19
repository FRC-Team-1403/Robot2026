package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;

public class InSpinShootCommand extends Command {
    private final Indexer m_indexer;
    private final Spindexer m_spindexer;
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;

    private final double m_indexerRPM;
    private final double m_spindexerRPM;
    private final double m_shooterRPM;
    private final double m_hoodAngle;

    public InSpinShootCommand(
            Indexer indexer,
            Spindexer spindexer,
            Shooter shooter,
            ShooterHood hood,
            double indexerRPM,
            double spindexerRPM,
            double shooterRPM,
            double hoodAngle) {
        m_indexer = indexer;
        m_spindexer = spindexer;
        m_shooter = shooter;
        m_shooterHood = hood;

        m_indexerRPM = indexerRPM;
        m_spindexerRPM = spindexerRPM;
        m_shooterRPM = shooterRPM;
        m_hoodAngle = hoodAngle;


        addRequirements(indexer, spindexer, shooter,hood);
    }

    @Override
    public void initialize() {
        m_shooter.setFlywheelTargetRPM(m_shooterRPM);
        m_shooterHood.setSetpoint(m_hoodAngle);
    }

    @Override
    public void execute() {
        if (m_shooter.isFlywheelAtSpeed()&&m_shooterHood.atSetpoint()) {
            m_indexer.setIndexerRPM(m_indexerRPM);
            m_spindexer.setSpindexerRPM(m_spindexerRPM);
        }
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
}