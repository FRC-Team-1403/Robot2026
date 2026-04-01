package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.Blackbox;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TurretTrackingCommand extends Command {

  private final Turret m_turret;
  private final Supplier<Pose2d> m_poseSupplier;

  public TurretTrackingCommand(Turret turret, Supplier<Pose2d> poseSupplier) {
    this.m_turret = turret;
    this.m_poseSupplier = poseSupplier;
    addRequirements(m_turret);
  }

  @Override
  public void execute() {
    Pose2d pose = m_poseSupplier.get();

    Translation2d turretPivotField = pose.getTranslation()
        .plus(Constants.Turret.kTurretOffset.rotateBy(pose.getRotation()));

    Translation2d target = Blackbox.getActiveTarget(pose);

    double deltaX = target.getX() - turretPivotField.getX();
    double deltaY = target.getY() - turretPivotField.getY();
    double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

    double robotHeading = pose.getRotation().getDegrees();
    double turretAngle = MathUtil.inputModulus(fieldAngleToGoal - robotHeading,
        Constants.Turret.kMinAngleDegrees, Constants.Turret.kMaxAngleDegrees);

    Logger.recordOutput("TurretCommand/RobotPose", pose);
    Logger.recordOutput("TurretCommand/TurretPivotField", new Pose2d(turretPivotField, new Rotation2d()));
    Logger.recordOutput("TurretCommand/ActiveTarget", new Pose2d(target, new Rotation2d()));
    Logger.recordOutput("TurretCommand/DeltaX", deltaX);
    Logger.recordOutput("TurretCommand/DeltaY", deltaY);
    Logger.recordOutput("TurretCommand/FieldAngleToGoal", fieldAngleToGoal);
    Logger.recordOutput("TurretCommand/TurretAngle", turretAngle);

    m_turret.setSetpoint(turretAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}