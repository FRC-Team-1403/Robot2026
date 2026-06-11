package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeWrist;

public class WristPowerCommand extends Command {
  private final IntakeWrist m_intakeWrist;
  private final double m_power;

  public WristPowerCommand(IntakeWrist intakeWrist, double power) {
    this.m_intakeWrist = intakeWrist;
    this.m_power = power;
    addRequirements(m_intakeWrist);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intakeWrist.setPower(m_power);
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