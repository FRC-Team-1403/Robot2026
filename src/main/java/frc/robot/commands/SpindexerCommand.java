package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;


public class SpindexerCommand extends Command {
  private double m_speed;
  private double m_speed2;
  private SpindexerSubsystem m_spindexer;

  public SpindexerCommand(double hopper, double shooter, SpindexerSubsystem subsystem) {
    m_speed = hopper;
    m_speed2 = shooter;
    m_spindexer = subsystem;

    addRequirements(subsystem);
  }
  
  @Override
  public void initialize() {
    m_spindexer.setHopperSpeed(-m_speed2);
    m_spindexer.setShooterSpeed(m_speed);
  }
  
  @Override
  public void execute() {
    
  }
  
  @Override
  public void end(boolean interrupted) {
    m_spindexer.setHopperSpeed(0);
    m_spindexer.setShooterSpeed(0);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }

}
