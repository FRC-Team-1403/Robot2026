package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretCommand extends Command {
    
    private final Turret m_turret;
    private final Vision m_vision;

    public TurretCommand(Turret m_turret, Vision m_vision) {
        this.m_turret = m_turret;
        this.m_vision = m_vision;
        
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        double targetAngleDegrees = Math.atan2(m_vision.getYPose(), m_vision.getXPose()) * (180.0/Math.PI);
        if (targetAngleDegrees < 0) {
            targetAngleDegrees += 360;
        }
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