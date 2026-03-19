package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;

public class WristCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final double m_speed;

  public WristCommand(IntakeWrist m_intakeWrist, double m_speed) {
    this.m_intakeWrist = m_intakeWrist;
    this.m_speed = m_speed;

    addRequirements(m_intakeWrist);
  }

  @Override
  public void initialize() {
  }
  
    @Override
  public void execute() {
    m_intakeWrist.setMotorOutput(0);
    
  }

  @Override
  public boolean isFinished() {
    // you should implement this
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeWrist.stopMotor();
  }
}