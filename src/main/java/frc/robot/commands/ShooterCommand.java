package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  private final Shooter m_shooter;
  private final double m_targetRPM;
  
  public ShooterCommand(Shooter shooter, double targetRPM) {
    m_shooter = shooter;
    m_targetRPM = targetRPM;
    addRequirements(shooter);
  }
  
  @Override
  public void initialize() {
    m_shooter.setTargetRPM(m_targetRPM);
  }
  
  @Override
  public void execute() {

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