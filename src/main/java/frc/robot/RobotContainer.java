// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.ShooterCommandPower;
import frc.robot.commands.ShooterCommandRPM;
import frc.robot.commands.ShooterHoodCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final Shooter m_shooter;
  private final ShooterHood m_shooterHood;

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  public RobotContainer() {
    m_shooter = new Shooter();
    m_shooterHood = new ShooterHood();
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
    //m_turret.setDefaultCommand(new TurretCommand(m_turret, m_vision));
    //m_operatorController.a().toggleOnTrue(new IntakeCommand(m_intake, m_intakeWrist));
    //m_operatorController.b().whileTrue(new ShooterCommand(m_shooter, m_indexer, m_spindexer, m_shooterHood, m_vision));

    m_driverController.rightTrigger().whileTrue(new ShooterCommandRPM(m_shooter, 1800));
    m_driverController.leftTrigger().whileTrue(new ShooterCommandPower(m_shooter, 0.2));
    m_driverController.a().whileTrue(new ShooterHoodCommand(m_shooterHood,10 ));



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}