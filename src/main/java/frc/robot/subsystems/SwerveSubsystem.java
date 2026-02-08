package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import java.io.File;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        try {
            File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveDirectory).createSwerveDrive(Constants.swerve.maxSpeed);

            SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        } catch (Exception e) {
            throw new RuntimeException("Failed to create YAGSL swerve drive", e);
        }
    }

    /**
     * Drive the robot with field-relative or robot-relative control
     * 
     * @param translation   Translation in m/s (x, y)
     * @param rotation      Rotation in rad/s
     * @param fieldRelative Whether to use field-relative control
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    /**
     * Drive using ChassisSpeeds (for PathPlanner)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    /**
     * Get current robot pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Reset robot pose
     */
    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Get current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get current heading
     */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * Zero the gyro
     */
    public void zeroHeading() {
        swerveDrive.zeroGyro();
    }

    /**
     * Lock wheels in X pattern
     */
    public void setX() {
        swerveDrive.lockPose();
    }

    /**
     * Stop all modules
     */
    public void stop() {
        swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }
}