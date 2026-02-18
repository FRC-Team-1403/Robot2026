package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class ShooterRPMCommand extends Command {
    private final IndexerSubsystem m_indexer;
    private final SpindexerSubsystem m_spindexer;
    private final double m_indexerRPM;
    private final double m_spindexerRPM;

    public ShooterRPMCommand(IndexerSubsystem indexer, SpindexerSubsystem spindexer, double indexerRPM, double spindexerRPM) {
        m_indexer = indexer;
        m_spindexer = spindexer;
        m_indexerRPM = indexerRPM;
        m_spindexerRPM = spindexerRPM;
        addRequirements(m_indexer, m_spindexer);
    }

    @Override
    public void initialize() {
        m_indexer.setIndexerTargetRPM(m_indexerRPM);
        m_spindexer.setSpindexerTargetRPM(m_spindexerRPM);
    }

    @Override
    public void execute() {
        m_indexer.setIndexerTargetRPM(m_indexerRPM);
        m_spindexer.setSpindexerTargetRPM(m_spindexerRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
        m_spindexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}