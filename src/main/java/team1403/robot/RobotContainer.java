// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package team1403.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.InSpinShootCommand;
import team1403.robot.commands.IntakeCommand;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.subsystems.Turret;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // private final SwerveSubsystem m_swerve;
  private final Intake m_intake;
  private final IntakeWrist m_intakeWrist;
  private final Indexer m_indexer;
  private final Spindexer m_spindexer;
  private final Shooter m_shooter;
  private final ShooterHood m_shooterHood;
  private final Turret m_turret;

  //vibration command
  private final Timer m_teleopTimer;

  @SuppressWarnings("all")
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    // m_swerve = TunerConstants.createDrivetrain();
    m_intake = new Intake();
    m_intakeWrist = new IntakeWrist();
    m_indexer= new Indexer();
    m_spindexer = new Spindexer();
    m_shooter = new Shooter();
    m_shooterHood = new ShooterHood();
    m_turret = new Turret();
    
    //for vibration command
    m_teleopTimer = new Timer();

    
    if (AutoBuilder.isConfigured())
      m_autoChooser =
          new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    else {
      m_autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser");
      DriverStation.reportError("Auto builder wasn't configured!", true);
    }

    SmartDashboard.putData("Auto Chooser", m_autoChooser.getSendableChooser());



    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_operatorController.a().toggleOnTrue(new IntakeCommand(m_intake, m_intakeWrist));
    m_operatorController.rightTrigger().whileTrue(new InSpinShootCommand(m_indexer, m_spindexer, m_shooter,1500,800,3000));


    //vibration command - untested 
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
      m_teleopTimer.reset();
      m_teleopTimer.start();
      }
    ));
    double[] shiftTimes = {140, 125, 105, 80, 55, 30};
    for (double shiftTime : shiftTimes) {
      final double time = 150 - shiftTime; // convert "time remaining" to "time elapsed"
      new Trigger(() -> DriverStation.isTeleopEnabled()
        && m_teleopTimer.get() <= time + 0.5
        && m_teleopTimer.get() >= time - 0.5)
        .onTrue(Commands.parallel(
            new ControllerVibrationCommand(m_driverController.getHID(), 0.6, 0.5).asProxy(),
            new ControllerVibrationCommand(m_operatorController.getHID(), 0.6, 0.5).asProxy()
        ));
}
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
