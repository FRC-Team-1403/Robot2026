package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommandPower extends Command {
  private final Shooter m_shooter;
  private final double m_targetPower;
  
  public ShooterCommandPower(Shooter shooter, double targetPower) {
    m_shooter = shooter;
    m_targetPower = targetPower;
    addRequirements(shooter);
  }
  
  @Override
  public void initialize() {
    
  }
  
  @Override
  public void execute() {
    m_shooter.setTargetPower(m_targetPower);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
