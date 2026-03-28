package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;

public class WristCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final double wristSpeed;

  public WristCommand(IntakeWrist m_intakeWrist, double wristSpeed) {
    this.m_intakeWrist = m_intakeWrist;
    this.wristSpeed = wristSpeed;
    addRequirements(m_intakeWrist);
  }

  @Override
  public void initialize() {
  }
  
    @Override
  public void execute() {
    //m_intakeWrist.setMotorOutput(wristSpeed);
    m_intakeWrist.setSetpoint(wristSpeed);
    
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