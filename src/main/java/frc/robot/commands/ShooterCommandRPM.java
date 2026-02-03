package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommandRPM extends Command {
  private final Shooter m_shooter;
  private final double m_flywheelTargetRPM;
  private final double m_rollerTargetRPM;
  
  public ShooterCommandRPM(Shooter shooter, double flywheelTargetRPM, double rollerTargetRPM) {
    m_shooter = shooter;
    m_flywheelTargetRPM = flywheelTargetRPM;
    m_rollerTargetRPM = rollerTargetRPM;
    addRequirements(shooter);
  }
  
  @Override
  public void initialize() {
    m_shooter.setFlywheelTargetRPM(m_flywheelTargetRPM);
    m_shooter.setRollerTargetRPM(m_rollerTargetRPM);
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
