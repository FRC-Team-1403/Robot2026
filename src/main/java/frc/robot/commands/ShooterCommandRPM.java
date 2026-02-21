package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommandRPM extends Command {
  private final Shooter m_shooter;
  private final double m_flywheelTargetRPM;
  
  public ShooterCommandRPM(Shooter shooter, double flywheelTargetRPM) {
    m_shooter = shooter;
    m_flywheelTargetRPM = flywheelTargetRPM;
    addRequirements(shooter);
  }
  
  @Override
  public void initialize() {
    m_shooter.setFlywheelTargetRPM(m_flywheelTargetRPM);
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