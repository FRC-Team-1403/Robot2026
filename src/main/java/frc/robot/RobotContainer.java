// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShooterRPMCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.SpindexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final IndexerSubsystem m_indexer;
  private final SpindexerSubsystem m_spindexer;
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    m_indexer = new IndexerSubsystem();
    m_spindexer = new SpindexerSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightTrigger().whileTrue(new ShooterRPMCommand(m_indexer, m_spindexer,5800,3600));
    m_driverController.leftTrigger().whileTrue(new IntakeCommand(m_intake, -1));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}