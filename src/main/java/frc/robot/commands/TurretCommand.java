package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretCommand extends Command {
    private final Turret m_turret;
    private final double m_targetAngle;

    public TurretCommand(Turret turret, double targetAngle) {
        m_turret = turret;
        m_targetAngle = targetAngle;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_turret.setSetpoint(m_targetAngle);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_turret.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return m_turret.atSetpoint();
    }
}