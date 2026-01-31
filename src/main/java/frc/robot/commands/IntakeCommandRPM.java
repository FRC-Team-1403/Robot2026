package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeTest;

public class IntakeCommandRPM extends Command {
  private final IntakeTest m_intake;
  private final double m_targetRPM;
  
  public IntakeCommandRPM(IntakeTest intake, double targetRPM) {
    m_intake = intake;
    m_targetRPM = targetRPM;
    addRequirements(intake);
  }
  
  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    m_shooter.setTargetRPM(m_targetRPM);
  }
  
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_shooter.stop();
    }
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
