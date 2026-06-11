package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;

public class IntakeCommand extends Command {
  private final Intake m_intake;
  private final double intakeSpeed;
  public IntakeCommand(Intake m_intake, double intakeSpeed) {
    this.m_intake = m_intake;
    this.intakeSpeed = intakeSpeed;

    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    m_intake.setIntakePower(intakeSpeed);
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