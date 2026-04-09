package team1403.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.Blackbox;

public class TurretShooterRampUpCommand extends Command {
    private final Turret m_turret;
    private final Shooter m_shooter;
    private final Supplier<Pose2d> m_pose;
    private final Supplier<ChassisSpeeds> m_chassisSupplier;
    private ChassisSpeeds m_lastVelocity;

    public TurretShooterRampUpCommand(
            Supplier<ChassisSpeeds> chassisSupplier,
            Turret turret,
            Shooter shooter,
            Supplier<Pose2d> pose) {
        m_chassisSupplier = chassisSupplier;
        m_turret = turret;
        m_shooter = shooter;
        m_pose = pose;
        m_lastVelocity = new ChassisSpeeds();
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        m_lastVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(m_chassisSupplier.get(), m_pose.get().getRotation());
    }

    @Override
    public void execute() {
        //Get updated values
        Pose2d robotPose = m_pose.get();
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(m_chassisSupplier.get(), robotPose.getRotation());

        //Transform robot into turret
        Translation2d turretPivotField = robotPose.getTranslation()
                .plus(Constants.Turret.kTurretOffset.rotateBy(robotPose.getRotation()));

        //Calculate current values
        Translation2d target = Blackbox.getActiveTarget(robotPose);
        double deltaX = target.getX() - turretPivotField.getX();
        double deltaY = target.getY() - turretPivotField.getY();
        double distance = Math.hypot(deltaX, deltaY);

        //Calculate future values
        double shotTime = lerp(Constants.Shooter.kTOFTable, distance);
        shotTime += Constants.Shooter.latency;
        double offsetX = shotTime * robotVelocity.vxMetersPerSecond;
        double offsetY = shotTime * robotVelocity.vyMetersPerSecond;
        Translation2d turret = Constants.Turret.kTurretOffset.rotateBy(robotPose.getRotation());
        double turretX = -1 * shotTime * robotVelocity.omegaRadiansPerSecond * turret.getY();
        double turretY =  1 * shotTime * robotVelocity.omegaRadiansPerSecond * turret.getX();
        Pose2d projectedPivot = new Pose2d(
                turretPivotField.getX() + offsetX + turretX,
                turretPivotField.getY() + offsetY + turretY,
                robotPose.getRotation());

        //Recompute distance
        double projDeltaX = target.getX() - projectedPivot.getX();
        double projDeltaY = target.getY() - projectedPivot.getY();
        double projectedDistance = Math.hypot(projDeltaX, projDeltaY);

        //Find and set turret
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(projDeltaY, projDeltaX));
        double robotHeading = projectedPivot.getRotation().getDegrees();
        double turretAngle = MathUtil.inputModulus(
                fieldAngleToGoal - robotHeading - Constants.Turret.rotationCorrectionOffset,
                Constants.Turret.kMinAngleDegrees,
                Constants.Turret.kMaxAngleDegrees);
        m_turret.setSetpoint(turretAngle);

        //LERP flywheel and hood speed/angle
        double flywheelRPM = lerp(Constants.Shooter.distanceTable, projectedDistance);

        //Set flywheel
        m_shooter.setFlywheelTargetRPM(flywheelRPM);

        //Calculate shot stability
        double translationDelta = 50 * Math.hypot(robotVelocity.vxMetersPerSecond - m_lastVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond - m_lastVelocity.vyMetersPerSecond);
        double rotationDelta = 50 * Math.abs(robotVelocity.omegaRadiansPerSecond - m_lastVelocity.omegaRadiansPerSecond);

        //Update values
        m_lastVelocity = robotVelocity;

        //Logging
        SmartDashboard.putNumber("Debug/distance", distance);
        Logger.recordOutput("TurretShooterRampUpCommand/RobotPose", robotPose);
        Logger.recordOutput("TurretShooterRampUpCommand/ProjectedPivot", projectedPivot);
        Logger.recordOutput("TurretShooterRampUpCommand/ActiveTarget", new Pose2d(target, new Rotation2d()));
        Logger.recordOutput("TurretShooterRampUpCommand/FieldAngleToGoal", fieldAngleToGoal);
        Logger.recordOutput("TurretShooterRampUpCommand/TurretAngle", turretAngle);
        Logger.recordOutput("TurretShooterRampUpCommand/ProjectedDistance", projectedDistance);
        Logger.recordOutput("TurretShooterRampUpCommand/FlywheelRPM", flywheelRPM);
        Logger.recordOutput("TurretShooterRampUpCommand/Transational Accel", translationDelta);
        Logger.recordOutput("TurretShooterRampUpCommand/Rotational Accel", rotationDelta);
        Logger.recordOutput("TurretShooterRampUpCommand/Turret Posistion", new Pose2d(turretPivotField.getX(), turretPivotField.getY(), new Rotation2d(turretAngle*Math.PI/180).plus(robotPose.getRotation())));
    }

    //Should never run (Default Command)
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    //Should never run (Default Command)
    @Override
    public boolean isFinished() {
        return false;
    }

    //LERP helper method
    public static double lerp(double[][] table, double input) {
        if (input <= table[0][0]) return table[0][1];
        if (input >= table[table.length - 1][0]) return table[table.length - 1][1];

        for (int i = 0; i < table.length - 1; i++) {
            if (input >= table[i][0] && input < table[i + 1][0]) {
                double t = (input - table[i][0]) / (table[i + 1][0] - table[i][0]);
                return table[i][1] + t * (table[i + 1][1] - table[i][1]);
            }
        }

        return table[table.length - 1][1];
    }
}