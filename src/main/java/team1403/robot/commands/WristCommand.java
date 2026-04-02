package team1403.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeWrist;

public class WristCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final double m_targetSetpoint;

  public WristCommand(IntakeWrist m_intakeWrist, double targetSetpoint) {
    this.m_intakeWrist = m_intakeWrist;
    this.m_targetSetpoint = targetSetpoint;
    addRequirements(m_intakeWrist);
  }

  @Override
  public void initialize() {
  }

  @Override

  public void execute() {
    m_intakeWrist.setSetpoint(m_targetSetpoint);
    
  }

  @Override
  public boolean isFinished() {
    return m_intakeWrist.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
  }
}