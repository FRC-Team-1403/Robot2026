package team1403.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeWrist;

public class WristCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final double m_targetSetpoint;
  private final SlewRateLimiter m_slewRateLimiter;

  public WristCommand(IntakeWrist m_intakeWrist, double targetSetpoint) {
    this.m_intakeWrist = m_intakeWrist;
    this.m_targetSetpoint = targetSetpoint;
    this.m_slewRateLimiter = new SlewRateLimiter(Constants.IntakeWrist.kSlewRate);
    addRequirements(m_intakeWrist);
  }

  @Override
  public void initialize() {
    m_slewRateLimiter.reset(m_intakeWrist.getSetpoint());
  }

  @Override
  public void execute() {
    double slewedSetpoint = m_slewRateLimiter.calculate(m_targetSetpoint);
    m_intakeWrist.setSetpoint(slewedSetpoint);
  }

  @Override
  public boolean isFinished() {
    return m_intakeWrist.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeWrist.stopMotor();
  }
}