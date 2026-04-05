// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package team1403.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.DefaultSwerveCommand;
import team1403.robot.commands.DriveWheelCharacterization;
import team1403.robot.commands.InSpinShootCommand;
import team1403.robot.commands.InSpinShootCommandTesting;
import team1403.robot.commands.IntakeCommand;
import team1403.robot.commands.LERPShooter;
import team1403.robot.commands.WristCommand;
import team1403.robot.commands.WristPowerCommand;
import team1403.robot.commands.WristWiggleCommand;
import team1403.robot.commands.auto.AutoHelper;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.subsystems.Turret;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;
import team1403.robot.util.CougarUtil;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Intake m_intake;
  private final IntakeWrist m_intakeWrist;
  private final Turret m_turret;
  private final Indexer m_indexer;
  private final Spindexer m_spindexer;
  private final Shooter m_shooter;
  private final ShooterHood m_shooterHood;
  private final SwerveSubsystem m_swerve;

  private final Timer m_teleopTimer;

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {

    m_swerve = TunerConstants.createDrivetrain();
    m_turret = new Turret();
    m_intake = new Intake();
    m_intakeWrist = new IntakeWrist();
    m_indexer= new Indexer();
    m_spindexer = new Spindexer();
    m_shooter = new Shooter();
    m_shooterHood = new ShooterHood();
    
    //for vibration command
    m_teleopTimer = new Timer();

    //autonomous coomands  
    NamedCommands.registerCommand("Intake Command", new IntakeCommand(m_intake, 1));
    NamedCommands.registerCommand("Shoot Command", new LERPShooter(() -> m_swerve.getState().Speeds,m_turret,m_indexer,m_spindexer,m_shooter,m_shooterHood,m_swerve::getPose,() -> 1.0));
    NamedCommands.registerCommand("Wrist Wiggle Command", new WristWiggleCommand(m_intakeWrist, m_intake));
    NamedCommands.registerCommand("IntakeWrist Down Command", new WristCommand(m_intakeWrist, Constants.IntakeWrist.downPos));
 
    if (AutoBuilder.isConfigured())
      m_autoChooser =
          new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    else {
      m_autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser");
      DriverStation.reportError("Auto builder wasn't configured!", true);
    }

    SmartDashboard.putData("Auto Chooser", m_autoChooser.getSendableChooser());

    //avoid cluttering up auto chooser at competitions and to not accidently click
    if (Constants.ENABLE_SYSID) {
      m_autoChooser.addOption("Swerve SysID QF", m_swerve.sysIdQuasistatic(Direction.kForward));
      m_autoChooser.addOption("Swerve SysID QR", m_swerve.sysIdQuasistatic(Direction.kReverse));
      m_autoChooser.addOption("Swerve SysID DF", m_swerve.sysIdDynamic(Direction.kForward));
      m_autoChooser.addOption("Swerve SysID DR", m_swerve.sysIdDynamic(Direction.kReverse));
      m_autoChooser.addOption("Drive Wheel Characterization", new DriveWheelCharacterization(m_swerve));
    }

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
    //testing command for elastic
    //m_operatorController.rightBumper().whileTrue(new InSpinShootCommandTesting(m_indexer, m_spindexer, m_shooter,m_shooterHood, 0 ,0, 0, 0));
    
    //Shooting Command
    //m_shooter.setDefaultCommand(new LERPShooter(() -> m_swerve.getState().Speeds, m_turret, m_indexer, m_spindexer, m_shooter, m_shooterHood, m_swerve::getPose, () -> m_operatorController.getHID().getRightTriggerAxis()));
    
    RobotModeTriggers.teleop().whileTrue(
      new LERPShooter(() -> m_swerve.getState().Speeds, m_turret, m_indexer, m_spindexer, m_shooter, m_shooterHood, m_swerve::getPose, () -> m_operatorController.getHID().getRightTriggerAxis())
    );
    
    //Intake Rollers
    m_operatorController.leftTrigger().whileTrue(new IntakeCommand(m_intake, 1));
    
    //Manual Wrist
    m_operatorController.y().whileTrue(new WristPowerCommand(m_intakeWrist, 0.3)); 
    m_operatorController.a().whileTrue(new WristPowerCommand(m_intakeWrist, -0.3)); 
    
    //Wiggle
    m_operatorController.leftBumper().whileTrue(new WristWiggleCommand(m_intakeWrist, m_intake));

    //Setpoint Wrist
    m_operatorController.povUp().onTrue(new WristCommand(m_intakeWrist, Constants.IntakeWrist.upPos)); 
    m_operatorController.povDown().onTrue(new WristCommand(m_intakeWrist, Constants.IntakeWrist.downPos));


    //swerve buttons 
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve, 
        () -> -m_driverController.getLeftX(),               //horozontal
        () -> -m_driverController.getLeftY(),               //vertical
        () -> -m_driverController.getRightX(),              //rotational 
        () -> m_driverController.getHID().getXButton(),     //x-mode  
        () -> false,                                        //robot relative  
        () -> m_driverController.getRightTriggerAxis(),     //acceleration
        () -> m_driverController.getLeftTriggerAxis(),      //snipping mode (slow down)-> m_operatorController.getRightTriggerAxis()>0.3
        () ->false,                                         // Auto Aim
        () -> m_driverController.getHID().getBButton()      // reset gyro
    ));

    // //vibration command - not tested 
    // RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
    // m_teleopTimer.reset();
    // m_teleopTimer.start();
    // }));

    // // 140 - shiftTime = elapsed time when shift occurs
    // // shift times remaining: 130, 105, 80, 55, 30
    // // elapsed equivalents:    10, 35, 60, 85
    // double[] shiftTimesElapsed = {10, 35, 60, 85};

    // for (double time : shiftTimesElapsed) {
    //   // driver: 5 seconds early
    //   new Trigger(() -> DriverStation.isTeleopEnabled()
    //     && m_teleopTimer.get() >= (time - 5) - 0.5
    //     && m_teleopTimer.get() <= (time - 5) + 0.5)
    //     .onTrue(new ControllerVibrationCommand(m_driverController.getHID(), 0.6, 0.5).asProxy()
    //   );

    //   // operator: 3 seconds early
    //   new Trigger(() -> DriverStation.isTeleopEnabled()
    //     && m_teleopTimer.get() >= (time - 3) - 0.5
    //     && m_teleopTimer.get() <= (time - 3) + 0.5)
    //     .onTrue(new ControllerVibrationCommand(m_operatorController.getHID(), 0.6, 0.5).asProxy()
    //   );
    // }

    
    m_autoChooser.addOption("AUTO ALIGN STATIONARY CENTER SHOOT SHOOTER FACES BACKWARDS", AutoHelper.getStationaryCenterShootAutoAlign(m_swerve));
    m_autoChooser.addOption("MIDDLE HUB DEPOT END TRENCH SHOOTER FACES BACKWARDS", AutoHelper.getMiddleHubDepotEndTrench(m_swerve));
    m_autoChooser.addOption("MIDDLE HUB DEPOT END HUB SHOOTER FACES BACKWARDS", AutoHelper.getMiddleHubDepotEndHub(m_swerve));
    m_autoChooser.addOption("LEFT TRENCH DOUBLE SWEEP INTAKE FACES FORWARD", AutoHelper.getLeftTrenchDoubleSweep(m_swerve));
    m_autoChooser.addOption("RIGHT TRENCH DOUBLE SWEEP INTAKE FACES FORWARD", AutoHelper.getRightTrenchDoubleSweep(m_swerve));
    m_autoChooser.addOption("LEFT TRENCH DOUBLE SWEEP END DEPOT INTAKE FACES FORWARD", AutoHelper.getLeftTrenchDoubleSweepDepot(m_swerve));
    m_autoChooser.addOption("LEFT BUMP DOUBLE SWEEP END MIDDLE INTAKE FACES FORWARD", AutoHelper.getLeftBumpDoubleSweepMiddle(m_swerve));
    m_autoChooser.addOption("LEFT BUMP DOUBLE SWEEP END DEPOT INTAKE FACES FORWARD", AutoHelper.getLeftBumpDoubleSweepDepot(m_swerve));

    
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
