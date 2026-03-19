package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;

public class IntakeCommand extends Command {
  private final Intake m_intake;
  private final IntakeWrist m_wrist;

  public IntakeCommand(Intake m_intake, IntakeWrist wrist) {
    this.m_intake = m_intake;
    this.m_wrist = wrist;

    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    if (Math.abs(m_wrist.getWristAngle() - Constants.IntakeWrist.kDeployedAngle) < 0.5) {
      m_intake.setIntakePower(Constants.Intake.rollerPower);
    }
    else {
      m_intake.setIntakePower(0);
    }
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