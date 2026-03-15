package team1403.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override public void disabledInit()    {}
  @Override public void disabledPeriodic(){}

  @Override
  public void teleopInit() {
    m_robotContainer.onTeleopInit();
  }

  @Override public void teleopPeriodic()  {}
  @Override public void autonomousInit()  {}
  @Override public void autonomousPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override public void testPeriodic()       {}
  @Override public void simulationInit()     {}
  @Override public void simulationPeriodic() {}
}
