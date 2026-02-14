package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeKrakenSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeKrakenCommand extends Command {
    private final IntakeKrakenSubsystem m_subsystem;
    private double m_rpm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

    public IntakeKrakenCommand(IntakeKrakenSubsystem subsystem, double rpm) {
    m_subsystem = subsystem;
    m_rpm = rpm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_subsystem.setVelocity(m_rpm); //change

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.setVelocity(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
