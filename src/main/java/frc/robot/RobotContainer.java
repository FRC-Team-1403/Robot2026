// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  private final Turret m_turret;

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  public RobotContainer() {
    m_turret = new Turret();
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

    m_driverController.a().whileTrue(new TurretCommand(m_turret, 0));
    m_driverController.x().whileTrue(new TurretCommand(m_turret, 90));
    m_driverController.b().whileTrue(new TurretCommand(m_turret, -90));
    m_driverController.y().whileTrue(new TurretCommand(m_turret, 180));




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