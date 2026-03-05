package frc.robot.Intake.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Intake.wrist.IntakeWrist;

public class IntakeCommand extends Command {
  private final Intake m_intake;
  private final IntakeWrist m_intakeWrist;
  private final double m_targetRPM;

  public IntakeCommand(Intake intake, IntakeWrist intakeWrist, double targetRPM) {
    m_intake = intake;
    m_intakeWrist = intakeWrist;
    m_targetRPM = targetRPM;

    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (m_intakeWrist.getWristAngle() > Constants.Intake.kWristRPMStartAngle) {
      m_intake.setIntakeRPM(m_targetRPM);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
}
