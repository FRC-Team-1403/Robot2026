package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;

public class IntakeCommand extends Command {
  private final Intake m_intake;
  private final IntakeWrist m_intakeWrist;
  //private final SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(0.3);

  public IntakeCommand(Intake m_intake, IntakeWrist m_intakeWrist) {
    this.m_intake = m_intake;
    this.m_intakeWrist = m_intakeWrist;

    addRequirements(m_intake, m_intakeWrist);
  }

  @Override
  public void initialize() {
    m_intakeWrist.setSetpoint(Constants.IntakeWrist.kDeployedAngle);
  }
  
    @Override
  public void execute() {
    if (m_intakeWrist.getWristAngle() > Constants.IntakeWrist.wristPowerStartAngle) {
      m_intake.setIntakePower(Constants.Intake.rollerPower);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_intakeWrist.setSetpoint(Constants.IntakeWrist.kStowedAngle);
  }
}