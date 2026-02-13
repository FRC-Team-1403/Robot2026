// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Vision;

public class RobotContainer {
  private final Turret m_turret;
  private final Vision m_vision;

  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Driver.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  public RobotContainer() {
    m_turret = new Turret();
    m_vision = new Vision();
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().whileTrue(new TurretCommand(m_turret, m_vision));
    m_turret.setDefaultCommand(new TurretCommand(m_turret, m_vision));

  }

  public Command getAutonomousCommand() {
    return new TurretCommand(m_turret, m_vision);
  }
}
