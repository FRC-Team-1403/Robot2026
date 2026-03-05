package frc.robot.Intake.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class IntakeWristCommand extends Command {
  private final IntakeWrist m_intakeWrist;

  public IntakeWristCommand(IntakeWrist intakeWrist) {
    m_intakeWrist = intakeWrist;

    addRequirements(m_intakeWrist);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeWrist.setSetpoint(Constants.IntakeWrist.kStowedAngle);
  }
}
