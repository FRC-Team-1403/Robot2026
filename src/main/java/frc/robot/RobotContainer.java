package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.OperatorConstants;
import frc.robot.vision.Vision;

public class RobotContainer {
  private final Vision m_vision;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private double m_xPosition = 8.0;
  private double m_yPosition = 4.0;
  private double m_rotation = 0.0;

  public RobotContainer() {
    m_vision = new Vision();
    configureBindings();
    
    m_vision.setDefaultCommand(
      new RunCommand(() -> {
        double xSpeed = -m_driverController.getLeftY() * 0.05;
        double ySpeed = -m_driverController.getLeftX() * 0.05;
        double rotSpeed = -m_driverController.getRightX() * 2.0;

        m_xPosition += xSpeed;
        m_yPosition += ySpeed;
        m_rotation += rotSpeed;

        Pose2d currentPose = new Pose2d(m_xPosition, m_yPosition, Rotation2d.fromDegrees(m_rotation));
        m_vision.updatePose(currentPose);
        m_vision.setSimPose(currentPose);
      }, m_vision)
    );
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}