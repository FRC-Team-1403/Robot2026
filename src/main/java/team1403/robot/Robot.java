package team1403.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Timer m_teleopTimer;
  private CommandXboxController m_driverController;
  private CommandXboxController m_operatorController;

  private final RobotContainer m_robotContainer;

  private final double[] shiftTimesElapsed = {10, 35, 60, 85};
  private final boolean[] driverTriggered = new boolean[shiftTimesElapsed.length];
  private final boolean[] operatorTriggered = new boolean[shiftTimesElapsed.length];

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/matchlogs"));
    Logger.start();

    m_robotContainer = new RobotContainer();
    m_teleopTimer = new Timer();

    m_driverController = new CommandXboxController(Constants.Driver.kDriverControllerPort);
    m_operatorController = new CommandXboxController(Constants.Operator.kOperatorControllerPort);

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double now = m_teleopTimer.get();

    // for (int i = 0; i < shiftTimesElapsed.length; i++) {
    //   double time = shiftTimesElapsed[i];

    //   // driver: 5 seconds early
    //   if (!driverTriggered[i] && now >= time - 5) {
    //     driverTriggered[i] = true;
    //     new ControllerVibrationCommand(
    //       m_driverController, 0.28, 1.0
    //     ).schedule();
    //   }

    //   // operator: 3 seconds early
    //   if (!operatorTriggered[i] && now >= time - 3) {
    //     operatorTriggered[i] = true;
    //     new ControllerVibrationCommand(
    //       m_operatorController, 0.28, 1.0
    //     ).schedule();
    //   }
    // }

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    m_teleopTimer.reset();
    m_teleopTimer.start();

    for (int i = 0; i < shiftTimesElapsed.length; i++) {
      driverTriggered[i] = false;
      operatorTriggered[i] = false;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
