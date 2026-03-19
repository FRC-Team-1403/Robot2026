package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeWrist;

public class WristCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final BooleanSupplier isDown;

  public WristCommand(IntakeWrist m_intakeWrist, BooleanSupplier isDown) {
    this.m_intakeWrist = m_intakeWrist;
    this.isDown = isDown;

    addRequirements(m_intakeWrist);
  }

  @Override
  public void initialize() {
  }
  
    @Override
  public void execute() {
    if (isDown.getAsBoolean()) {
      m_intakeWrist.setSetpoint(Constants.IntakeWrist.kDeployedAngle);
    }
    else {
      m_intakeWrist.setSetpoint(Constants.IntakeWrist.kStowedAngle);
    }    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeWrist.stopMotor();
  }
}