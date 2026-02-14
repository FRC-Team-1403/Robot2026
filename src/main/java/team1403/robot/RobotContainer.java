// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package team1403.robot;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.DefaultSwerveCommand;
import team1403.robot.commands.GroundIntakeCommandPower;
import team1403.robot.commands.GroundIntakeCommandRPM;

import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.GroundIntake;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class RobotContainer {
  private final SwerveSubsystem m_swerve;
  private final GroundIntake m_groundIntake;
  

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;


  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);

    Blackbox.init();
    m_swerve = TunerConstants.createDrivetrain();
    m_groundIntake = new GroundIntake();
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);

    if(TunerConstants.SWERVE_DEBUG_MODE) {
      SmartDashboard.putData("Swerve Drive", m_swerve);
    }

    configureBindings();
  }

  private void configureBindings() {
   
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

    m_driverController.leftTrigger().whileTrue(new GroundIntakeCommandPower(m_groundIntake, -0.5));
    m_driverController.rightTrigger().whileTrue(new GroundIntakeCommandRPM(m_groundIntake, 1000));

    if(Constants.ENABLE_SYSID){
    m_driverController.a().whileTrue(m_swerve.sysIdQuasistatic(Direction.kForward));
    m_driverController.b().whileTrue(m_swerve.sysIdQuasistatic(Direction.kReverse));
    m_driverController.rightBumper().whileTrue(m_swerve.sysIdDynamic(Direction.kForward));
    m_driverController.leftBumper().whileTrue(m_swerve.sysIdDynamic(Direction.kReverse));
    }
    
  }
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}