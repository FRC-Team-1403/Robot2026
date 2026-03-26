package team1403.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.Blackbox;

public class TurretTrackingCommand extends Command {
    private final Turret m_turret;
    private final Supplier<Pose2d> m_pose;
    private final Supplier<ChassisSpeeds> m_chassisSpeeds;

    public TurretTrackingCommand(
            Turret turret,
            Supplier<Pose2d> pose,
            Supplier<ChassisSpeeds> chassisSpeeds) {
        m_turret = turret;
        m_pose = pose;
        m_chassisSpeeds = chassisSpeeds;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = m_chassisSpeeds.get();
        Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        Pose2d currentPose = m_pose.get().transformBy(new Transform2d(Constants.Turret.kTurretOffset, new Rotation2d()));

        Translation2d futurePosition = currentPose.getTranslation().plus(robotVelocity.times(Constants.Shooter.kLatencyCompensation));
        Pose2d futurePose = new Pose2d(futurePosition, currentPose.getRotation());

        double diffX = Blackbox.getActiveTarget(futurePose).getX() - futurePosition.getX();
        double diffY = Blackbox.getActiveTarget(futurePose).getY() - futurePosition.getY();
        double distance = Math.hypot(diffX, diffY);

        Translation2d targetDirection = new Translation2d(diffX, diffY).div(distance);
        double baselineToF = LERPShooter.lerp(Constants.Shooter.kTOFTable, distance);
        double baselineHorizontalVelocity = distance / baselineToF;

        Translation2d targetVelocityVector = targetDirection.times(baselineHorizontalVelocity);
        Translation2d shotVector = targetVelocityVector.minus(robotVelocity);

        double fieldRelativeShotAngle = shotVector.getAngle().getDegrees();
        double robotYaw = currentPose.getRotation().getDegrees();
        double turretAngle = fieldRelativeShotAngle - robotYaw;

        m_turret.setSetpoint(turretAngle);

        Logger.recordOutput("TurretTracking/Field Relative Shot Angle", fieldRelativeShotAngle);
        Logger.recordOutput("TurretTracking/Turret Setpoint", turretAngle);
        Logger.recordOutput("TurretTracking/Distance", distance);
        Logger.recordOutput("TurretTracking/Baseline Horizontal Velocity", baselineHorizontalVelocity);
        Logger.recordOutput("TurretTracking/Required Horizontal Velocity", shotVector.getNorm());
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}