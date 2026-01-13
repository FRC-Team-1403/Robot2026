package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretCommand extends Command {
    
    private final Turret m_turret;
    private final double targetAngleDegrees;

    public TurretCommand(Turret turret, double targetAngleDegrees) {
        this.m_turret = turret;
        this.targetAngleDegrees = targetAngleDegrees;
        
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_turret.setSetpoint(targetAngleDegrees);
    }

    @Override
    public void execute() {
       
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
      
        return m_turret.atSetpoint();
    }
}