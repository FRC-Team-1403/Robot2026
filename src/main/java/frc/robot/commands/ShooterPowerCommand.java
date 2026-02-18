package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class ShooterPowerCommand extends Command {
    private final IndexerSubsystem m_indexer;
    private final SpindexerSubsystem m_spindexer;
    private final double m_indexerDutyCycle;
    private final double m_spindexerDutyCycle;

    public ShooterPowerCommand(IndexerSubsystem indexer, SpindexerSubsystem spindexer, double indexerDutyCycle, double spindexerDutyCycle) {
        m_indexer = indexer;
        m_spindexer = spindexer;
        m_indexerDutyCycle = indexerDutyCycle;
        m_spindexerDutyCycle = spindexerDutyCycle;
        addRequirements(m_indexer, m_spindexer);
    }

    @Override
    public void initialize() {
        m_indexer.setIndexerTargetPower(m_indexerDutyCycle);
        m_spindexer.setSpindexerTargetPower(m_spindexerDutyCycle);
    }

    @Override
    public void execute() {
        m_indexer.setIndexerTargetPower(m_indexerDutyCycle);
        m_spindexer.setSpindexerTargetPower(m_spindexerDutyCycle);
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