// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package team1403.robot;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.robot.Constants.Swerve;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.DefaultSwerveCommand;
import team1403.robot.commands.GroundIntakeCommandPower;
import team1403.robot.commands.GroundIntakeCommandRPM;

import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.GroundIntake;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;
import team1403.robot.vision.AprilTagCamera;
import team1403.robot.vision.ITagCamera;
import team1403.robot.vision.VisionSimUtil;

public class RobotContainer {
  private final SwerveSubsystem m_swerve;
  private final GroundIntake m_groundIntake;
  private List<ITagCamera> m_cameras;
  

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;


  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);

    Blackbox.init();

    //*****if code still doesn't work try running this tmrw
    
    //m_autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    //m_autoChooser.addDefaultOption("None", Commands.none());

    if (AutoBuilder.isConfigured())
      m_autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    else {
      m_autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser");
      DriverStation.reportError("Auto builder wasn't configured!", true);
    }

    SmartDashboard.putData("Auto Chooser", m_autoChooser.getSendableChooser());

    m_swerve = TunerConstants.createDrivetrain();
    m_groundIntake = new GroundIntake();
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);

    // m_cameras = List.of(
    //   new AprilTagCamera("ThriftyCamera1.0", () -> Constants.Vision.kCameraTransfromThriftyCamera1, m_swerve::getPose),
    //   new AprilTagCamera("ThriftyCamera2.0", () -> Constants.Vision.kCameraTransfromThriftyCamera2, m_swerve::getPose),
    //   new AprilTagCamera("ThriftyCamera3.0", () -> Constants.Vision.kCameraTransfromThriftyCamera3, m_swerve::getPose),
    //   new AprilTagCamera("ThriftyCamera4.0", () -> Constants.Vision.kCameraTransfromThriftyCamera4, m_swerve::getPose)
    // );

    // VisionSimUtil.initVisionSim();
    // m_swerve.setCameras(m_cameras);

    if(TunerConstants.SWERVE_DEBUG_MODE) {
      SmartDashboard.putData("Swerve Drive", m_swerve);
    }

    configureBindings();
  }

  private void configureBindings() {
   
    if(!Constants.ENABLE_SYSID){
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve,
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getRightX(),
        () -> m_driverController.getHID().getPOV() == 180,
        () -> m_driverController.getHID().getPOV() == 0,
        () -> m_driverController.getHID().getAButton(),
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftTriggerAxis()));

    Command vibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);
    Command opVibrationCmd = new ControllerVibrationCommand(m_operatorController.getHID(), 0.28, 1);

    //**** change from triger to bumper to not mess with swervee
    m_driverController.leftBumper().whileTrue(new GroundIntakeCommandPower(m_groundIntake, -0.5));
    //m_driverController.rightBumper().whileTrue(new GroundIntakeCommandRPM(m_groundIntake, 30));
    }


    if(Constants.ENABLE_SYSID){
      m_driverController.rightTrigger().whileTrue(m_swerve.sysIdQuasistatic(Direction.kForward));
      m_driverController.leftTrigger().whileTrue(m_swerve.sysIdQuasistatic(Direction.kReverse));
      m_driverController.rightBumper().whileTrue(m_swerve.sysIdDynamic(Direction.kForward));
      m_driverController.leftBumper().whileTrue(m_swerve.sysIdDynamic(Direction.kReverse));
    }
    
  }
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}