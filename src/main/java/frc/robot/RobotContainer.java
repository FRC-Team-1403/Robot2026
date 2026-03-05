// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Spindexer.Spindexer;
import frc.robot.Turret.Turret;
import frc.robot.Turret.TurretCommand;
import frc.robot.Vision.Vision;
import frc.robot.Indexer.Indexer;
import frc.robot.Intake.rollers.Intake;
import frc.robot.Intake.rollers.IntakeCommand;
import frc.robot.Intake.wrist.IntakeWrist;
import frc.robot.Intake.wrist.IntakeWristCommand;
import frc.robot.Shooter.ShooterCommand;
import frc.robot.Shooter.flywheel.Shooter;
import frc.robot.Shooter.hood.ShooterHood;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final IntakeWrist m_IntakeWrist;
  private final Intake m_Intake;
  private final Shooter m_Shooter;
  private final ShooterHood m_ShooterHood;
  private final Spindexer m_Spindexer;
  private final Indexer m_Indexer;
  private final Vision m_Vision;
  private final Turret m_Turret;

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_IntakeWrist = new IntakeWrist();
    m_Intake = new Intake();
    m_Shooter = new Shooter();
    m_ShooterHood = new ShooterHood();
    m_Spindexer = new Spindexer();
    m_Indexer = new Indexer();
    m_Vision = new Vision();
    m_Turret = new Turret();

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
    m_Intake.setDefaultCommand(new IntakeCommand(m_Intake, m_IntakeWrist, Constants.Intake.kRPM));
    m_IntakeWrist.setDefaultCommand(new IntakeWristCommand(m_IntakeWrist));
    m_Shooter.setDefaultCommand(new ShooterCommand(m_Shooter, m_Indexer, m_Spindexer, m_ShooterHood, m_Vision));
    m_Turret.setDefaultCommand(new TurretCommand(m_Turret, m_Vision));
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
