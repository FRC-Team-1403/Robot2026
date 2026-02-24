// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootOnTheFly;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Vision;
// import frc.robot.commands.auto.AutoHelper;
// import frc.robot.swerve.SwerveSubsystem;
// import frc.robot.swerve.TunerConstants;

public class RobotContainer {
  //private final SwerveSubsystem m_swerve;

  private final Shooter m_shooter;
  private final ShooterHood m_hood;
  private final Turret m_turret;
  private final Indexer m_indexer;
  private final Spindexer m_spindexer;
  private final Vision m_vision;

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    //m_swerve = TunerConstants.createDrivetrain();

    m_shooter = new Shooter();
    m_hood = new ShooterHood();
    m_turret = new Turret();
    m_indexer = new Indexer();
    m_spindexer = new Spindexer();
    m_vision = new Vision();

    if (AutoBuilder.isConfigured())
      m_autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
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
    m_operatorController.rightTrigger().or(m_operatorController.leftTrigger()).whileTrue(
        new ShootOnTheFly(
            m_shooter,
            m_hood,
            m_turret,
            m_indexer,
            m_spindexer,
            m_vision,
            () -> null, // TODO: replace with m_swerve::getFieldRelativeSpeeds
            () -> null, // TODO: replace with m_swerve::getPose
            m_operatorController.rightTrigger(),
            m_operatorController.leftTrigger()
        )
    );
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