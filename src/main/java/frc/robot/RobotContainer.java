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
// import frc.robot.commands.auto.AutoHelper;
// import frc.robot.swerve.SwerveSubsystem;
// import frc.robot.swerve.TunerConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;

public class RobotContainer {
  //private final SwerveSubsystem m_swerve;
  private final Intake m_intake;
  private final IntakeWrist m_intakeWrist;

  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.Driver.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.Operator.kOperatorControllerPort);

  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    //m_swerve = TunerConstants.createDrivetrain();
    m_intake = new Intake();
    m_intakeWrist = new IntakeWrist();

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
    m_operatorController.a().toggleOnTrue(new IntakeCommand(m_intake, m_intakeWrist));
    
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