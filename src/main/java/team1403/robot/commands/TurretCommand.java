package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Turret;
import team1403.robot.vision.Vision;

import org.littletonrobotics.junction.Logger;

public class TurretCommand extends Command {

  private final Turret m_turret;
  private final Vision m_vision;

  public TurretCommand(Turret m_turret, Vision m_vision) {
    this.m_turret = m_turret;
    this.m_vision = m_vision;

    addRequirements(m_turret);
  }

  @Override
  public void execute() {
    if (m_vision.hasPose()) {
      Pose2d robotPose = m_vision.getPose2d();

      // Calculate angle to goal
      double deltaX = Constants.Vision.kGoalX - robotPose.getX();
      double deltaY = Constants.Vision.kGoalY - robotPose.getY();
      double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

      double robotHeading = robotPose.getRotation().getDegrees();
      double turretAngle = MathUtil.inputModulus(fieldAngleToGoal - robotHeading, -180, 180);

      // Log for debugging
      Logger.recordOutput("TurretCommand/RobotPose", robotPose);
      Logger.recordOutput(
          "TurretCommand/GoalPose",
          new Pose2d(Constants.Vision.kGoalX, Constants.Vision.kGoalY, new Rotation2d()));
      Logger.recordOutput("TurretCommand/DeltaX", deltaX);
      Logger.recordOutput("TurretCommand/DeltaY", deltaY);
      Logger.recordOutput("TurretCommand/FieldAngleToGoal", fieldAngleToGoal);
      Logger.recordOutput("TurretCommand/TurretAngle", turretAngle);

      m_turret.setSetpoint(turretAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
