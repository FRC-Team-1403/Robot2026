package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Swervejoystickcommand;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    
    private final SwerveDrive m_swerveDrive = new SwerveDrive();
    
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    
    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
    }
    
    private void configureDefaultCommands() {
        m_swerveDrive.setDefaultCommand(
            new Swervejoystickcommand(
                m_swerveDrive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                () -> true
            )
        );
    }
    
    private void configureBindings() {
        m_driverController.a().onTrue(
            m_swerveDrive.runOnce(() -> m_swerveDrive.zeroHeading())
        );
        
        m_driverController.x().whileTrue(
            m_swerveDrive.run(() -> m_swerveDrive.setX())
        );
        
        m_driverController.b().onTrue(
            m_swerveDrive.runOnce(() -> m_swerveDrive.resetEncoders())
        );
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}