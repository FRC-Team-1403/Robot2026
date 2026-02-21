package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommandPower extends Command {
  private final Shooter m_shooter;
  private final double m_flywheelTargetPower;
  
  public ShooterCommandPower(Shooter shooter, double flywheelTargetPower) {
    m_shooter = shooter;
    m_flywheelTargetPower = flywheelTargetPower;
    addRequirements(shooter);
  }
  
  @Override
  public void initialize() {
    m_shooter.setFlywheelTargetPower(m_flywheelTargetPower);
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