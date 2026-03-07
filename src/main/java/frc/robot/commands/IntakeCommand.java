package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;

public class IntakeCommand extends Command {
  private final Intake m_intake;
  private final IntakeWrist m_intakeWrist;
  private SlewRateLimiter ramp = new SlewRateLimiter(0.3); // power units per second

  public IntakeCommand(Intake m_intake, IntakeWrist m_intakeWrist) {
    this.m_intake = m_intake;
    this.m_intakeWrist = m_intakeWrist;

    addRequirements(m_intake, m_intakeWrist);
  }

  @Override
  public void initialize() {
    ramp.reset(0); 
  }

  @Override
  public void execute() {
    double targetPower = 1;
    double rampedPower = ramp.calculate(targetPower);
    m_intake.setIntakePower(rampedPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePower(0.2);
    m_intake.stop();
  }
}
