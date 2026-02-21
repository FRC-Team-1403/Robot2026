package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHood;

public class ShooterHoodCommand extends Command {
    private final ShooterHood m_shooterHood;
    private final double m_targetAngle;

    public ShooterHoodCommand(ShooterHood shooterHood, double targetAngle) {
        m_shooterHood = shooterHood;
        m_targetAngle = targetAngle;
        addRequirements(shooterHood);
    }

    @Override
    public void initialize() {
        m_shooterHood.setSetpoint(m_targetAngle);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_shooterHood.atSetpoint();
    }
}