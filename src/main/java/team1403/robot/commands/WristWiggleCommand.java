package team1403.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;

public class WristWiggleCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final Intake m_intake;
  private final SlewRateLimiter m_slewRateLimiter;
  private boolean m_goingUp;
  private double m_slewedSetpoint;

  public WristWiggleCommand(IntakeWrist m_intakeWrist, Intake m_intake) {
    this.m_intakeWrist = m_intakeWrist;
    this.m_intake = m_intake;
    this.m_slewRateLimiter = new SlewRateLimiter(Constants.IntakeWrist.kSlewRate);
    addRequirements(m_intakeWrist, m_intake);
  }

  @Override
  public void initialize() {
    m_goingUp = true;
    m_slewRateLimiter.reset(m_intakeWrist.getWristAngle());
    m_slewedSetpoint = m_intakeWrist.getWristAngle();
    m_intakeWrist.setSetpoint(50);
  }

  @Override
  public void execute() {
    if (m_intakeWrist.atSetpoint()) {
      m_goingUp = !m_goingUp;
      if (m_goingUp) {
        m_intakeWrist.setSetpoint(50);
      } else {
        m_intakeWrist.setSetpoint(95);
      }
    }

    m_slewedSetpoint = m_slewRateLimiter.calculate(m_intakeWrist.getSetpoint());
    m_intakeWrist.setSetpoint(m_slewedSetpoint);
    m_intake.setIntakePower(1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeWrist.stopMotor();
    m_intake.stop();
  }
}