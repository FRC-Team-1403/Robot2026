package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;

public class WristWiggleCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final Intake m_intake;
  private boolean m_goingUp;
  private double m_targetSetpoint;
  private final Timer m_timer;

  public WristWiggleCommand(IntakeWrist m_intakeWrist, Intake m_intake) {
    this.m_intakeWrist = m_intakeWrist;
    this.m_intake = m_intake;
    m_timer = new Timer();
    addRequirements(m_intakeWrist, m_intake);
  }

  @Override
  public void initialize() {
    m_goingUp = true;
    m_targetSetpoint = Constants.IntakeWrist.upPos;
    m_intakeWrist.setSetpoint(m_targetSetpoint);
    m_timer.restart();
  }

  @Override
  public void execute() {
    if (m_intakeWrist.atSetpoint() || m_timer.hasElapsed(1)) {
      if (m_goingUp) {
        m_goingUp = false;
        m_targetSetpoint = Constants.IntakeWrist.downPos;
      } else {
        m_goingUp = true;
        m_targetSetpoint = Constants.IntakeWrist.upPos;
      }
      m_intakeWrist.setSetpoint(m_targetSetpoint);
      m_timer.restart();
    }

    m_intake.setIntakePower(1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeWrist.setSetpoint(Constants.IntakeWrist.downPos);
    m_intake.stop();
  }
}