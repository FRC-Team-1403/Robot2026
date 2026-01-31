package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeTest;

public class IntakeCommandPower extends Command {
  private final IntakeTest m_intake;
  private final double m_targetPower;
  
  public IntakeCommandPower(IntakeTest intake, double targetPower) {
    m_intake = intake;
    m_targetPower = targetPower;
    addRequirements(intake);
  }
  
  @Override
  public void initialize() {
    
  }
  
  @Override
  public void execute() {
    m_intake.setTargetPower(m_targetPower);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
