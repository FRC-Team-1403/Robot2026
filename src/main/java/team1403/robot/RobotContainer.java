// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package team1403.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.DefaultSwerveCommand;
import team1403.robot.commands.DriveWheelCharacterization;
import team1403.robot.commands.InSpinShootCommand;
import team1403.robot.commands.IntakeCommand;
import team1403.robot.commands.WristCommand;
import team1403.robot.commands.auto.AutoHelper;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;
import team1403.robot.util.CougarUtil;
import team1403.robot.vision.Vision;


import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Intake m_intake;
  private final IntakeWrist m_intakeWrist;
  private final Indexer m_indexer;
  private final Spindexer m_spindexer;
  private final Shooter m_shooter;
  private final ShooterHood m_shooterHood;
  private final SwerveSubsystem m_swerve;
  private final Vision m_vision; 

  private final Timer m_teleopTimer;

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {

    m_swerve = TunerConstants.createDrivetrain();
    
    m_intake = new Intake();
    m_intakeWrist = new IntakeWrist();
    m_indexer= new Indexer();
    m_spindexer = new Spindexer();
    m_shooter = new Shooter();
    m_shooterHood = new ShooterHood();
    m_vision = new Vision();
    
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
    //Shooting Commands 
      //against hub 
    m_shooter.setDefaultCommand(new InSpinShootCommand(m_indexer, m_spindexer, m_shooter, m_shooterHood, 0, 0, 0, 0));
    m_operatorController.rightTrigger().whileTrue(new InSpinShootCommand(m_indexer, m_spindexer, m_shooter,m_shooterHood, 
                                    Constants.InSpinShoot.kIndexerRPM_hub ,Constants.InSpinShoot.kSpindexerRPM_hub,
                                    Constants.InSpinShoot.kShooterRPM_hub, Constants.InSpinShoot.kHoodAngle_hub));
      //against tower 
    m_operatorController.leftTrigger().whileTrue(new InSpinShootCommand(m_indexer, m_spindexer, m_shooter,m_shooterHood, 
                                    Constants.InSpinShoot.kIndexerRPM_tower ,Constants.InSpinShoot.kSpindexerRPM_tower,
                                    Constants.InSpinShoot.kShooterRPM_tower, Constants.InSpinShoot.kHoodAngle_tower));
      //feed depo side (left)
    m_operatorController.leftBumper().whileTrue(new InSpinShootCommand(m_indexer, m_spindexer, m_shooter,m_shooterHood, 
                                    Constants.InSpinShoot.kIndexerRPM_left ,Constants.InSpinShoot.kSpindexerRPM_left,
                                    Constants.InSpinShoot.kShooterRPM_left, Constants.InSpinShoot.kHoodAngle_left));
      //feed human player side (right) 
    m_operatorController.rightBumper().whileTrue(new InSpinShootCommand(m_indexer, m_spindexer, m_shooter,m_shooterHood, 
                                    Constants.InSpinShoot.kIndexerRPM_right ,Constants.InSpinShoot.kSpindexerRPM_right,
                                    Constants.InSpinShoot.kShooterRPM_right, Constants.InSpinShoot.kHoodAngle_right));

    
    m_operatorController.b().whileTrue(new IntakeCommand(m_intake, 1));
    m_operatorController.x().onTrue(new IntakeCommand(m_intake, 0));
    m_operatorController.povUp().whileTrue(new ParallelCommandGroup(new WristCommand(m_intakeWrist, -0.3), 
                                                                    new IntakeCommand(m_intake, 1)));
    m_operatorController.povDown().whileTrue(new WristCommand(m_intakeWrist, 0.3));
    


    //swerve buttons 
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve, 
        () -> -m_driverController.getLeftX(),               //horozontal
        () -> -m_driverController.getLeftY(),               //vertical
        () -> -m_driverController.getRightX(),              //rotational 
        () -> m_driverController.getHID().getPOV() == 180,  //x-mode  
        () -> m_driverController.getHID().getPOV() == 0,    //robot relative  
        () -> m_driverController.getRightTriggerAxis(),     //acceleration
        () -> m_driverController.getLeftTriggerAxis(),      //snipping mode (slow down)
        () -> m_driverController.getHID().getStartButton()
        ));

    
    // //NamedCommands.registerCommand("Intake Command", new IntakeCommand(m_intake));
    // NamedCommands.registerCommand("Stationary Shoot Command", 
    //         new InSpinShootCommand(m_indexer, m_spindexer, m_shooter, m_shooterHood, 
   // Constants.InSpinShoot.kAutoIndexerRPM, Constants.InSpinShoot.kAutoSpindexerRPM, 
    //Constants.InSpinShoot.kAutoShooterRPM, Constants.InSpinShoot.kAutoHoodAngle));


    m_autoChooser.addOption("STATIONARY SHOOT", AutoHelper.getStationaryShoot(m_swerve));
    m_autoChooser.addOption("HUMAN PLAYER", AutoHelper.getHumanPlayer(m_swerve));
    m_autoChooser.addOption("RIGHT SWEEP 1X THEN HUMAN PLAYER", AutoHelper.getCenterHuman(m_swerve));
    m_autoChooser.addOption("RIGHT SWEEP 2X THEN HUMAN PLAYER", AutoHelper.getRightSweep(m_swerve));
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
