package frc.robot.swerve;


import com.studica.frc.AHRS;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.swerve.SwerveHeadingCorrector;
//import team1403.lib.device.wpi.NavxAhrs;
//import com.kauailabs.navx.AHRS;

/**
 * Swerve drive subsystem - manages all 4 modules, gyro, and pose estimation.
 */
public class SwerveDrive extends SubsystemBase {

    // Hardware
    
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;
    private final AHRS m_gyro;
    
    // Tracking
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field;

    private SwerveHeadingCorrector m_headingCorrector = new SwerveHeadingCorrector();

    public SwerveDrive() {
        // Initialize all four swerve modules
        m_frontLeft = new SwerveModule(
            "Front Left",
            SwerveConstants.ModuleConstants.kFrontLeftDriveID,
            SwerveConstants.ModuleConstants.kFrontLeftSteerID,
            SwerveConstants.ModuleConstants.kFrontLeftEncoderID,
            SwerveConstants.ModuleConstants.kFrontLeftOffset,
            SwerveConstants.ModuleConstants.kFrontLeftDriveInverted
        );

        m_frontRight = new SwerveModule(
            "Front Right",
            SwerveConstants.ModuleConstants.kFrontRightDriveID,
            SwerveConstants.ModuleConstants.kFrontRightSteerID,
            SwerveConstants.ModuleConstants.kFrontRightEncoderID,
            SwerveConstants.ModuleConstants.kFrontRightOffset,
            SwerveConstants.ModuleConstants.kFrontRightDriveInverted
        );

        m_backLeft = new SwerveModule(
            "Back Left",
            SwerveConstants.ModuleConstants.kBackLeftDriveID,
            SwerveConstants.ModuleConstants.kBackLeftSteerID,
            SwerveConstants.ModuleConstants.kBackLeftEncoderID,
            SwerveConstants.ModuleConstants.kBackLeftOffset,
            SwerveConstants.ModuleConstants.kBackLeftDriveInverted
        );

        m_backRight = new SwerveModule(
            "Back Right",
            SwerveConstants.ModuleConstants.kBackRightDriveID,
            SwerveConstants.ModuleConstants.kBackRightSteerID,
            SwerveConstants.ModuleConstants.kBackRightEncoderID,
            SwerveConstants.ModuleConstants.kBackRightOffset,
            SwerveConstants.ModuleConstants.kBackRightDriveInverted
        );

        // Initialize gyro on MXP SPI port
        m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        
        // Initialize pose estimator at origin
        m_poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            getRotation2d(),
            getModulePositions(),
            new Pose2d()
        );

        // Field visualization for dashboard
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        // Configure PathPlanner for autonomous
        configurePathPlanner();

        // Zero gyro after 1 second delay (allows gyro to initialize)
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();
    }

    // ===== ADD THESE HELPER METHODS =====
    /**
    * Gets the current chassis speeds of the robot
    * @return Current robot-relative chassis speeds
    */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
    * Gets the current angular velocity from the gyro
    * @return Angular velocity in radians per second
    */
    private double getGyroAngularVelocity() {
        // Replace with your actual gyro object - example for Pigeon2:
        // return Math.toRadians(m_gyro.getRate());
        
        // Or for NavX:
        return Math.toRadians(m_gyro.getRate());
        //throw new UnsupportedOperationException("Implement getGyroAngularVelocity() for your gyro");
    }
    // ===== END HELPER METHODS =====
        /**
         * Configure PathPlanner for autonomous path following.
         */
    //TODO
    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,                    // Pose supplier
                this::resetPose,                  // Pose resetter
                this::getChassisSpeeds,           // Speeds supplier
                (speeds, feedforwards) -> driveRobotRelative(speeds),  // Drive method
                new PPHolonomicDriveController(
                    SwerveConstants.kTranslationPID,
                    SwerveConstants.kRotationPID
                ),
                config,
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner", e.getStackTrace());
        }
    }

    /**
     * Drive with X/Y speeds and rotation. Can be field-relative or robot-relative.
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            // Convert field-relative speeds to robot-relative
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, getRotation2d()
            );
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        // ===== HEADING CORRECTION ADDED HERE =====
        // Apply heading correction to maintain heading when driver isn't rotating
        // Get current robot state for heading correction
        ChassisSpeeds currentSpeeds = getChassisSpeeds(); // You'll need to implement this
        Rotation2d gyroAngle = getRotation2d();
        double gyroAngularVel = getGyroAngularVelocity(); // You'll need to implement this (radians/sec)
        
        // Update speeds with heading correction
        speeds = m_headingCorrector.update(
            speeds,
            currentSpeeds,
            gyroAngle,
            gyroAngularVel
        );
        // ===== END HEADING CORRECTION =====

        driveRobotRelative(speeds);
    }

    /**
     * Drive using robot-relative chassis speeds.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Discretize to compensate for loop time (reduces drift)
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Convert to individual module states
        SwerveModuleState[] moduleStates =
            SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        // Scale down if any wheel exceeds max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            SwerveConstants.kMaxSpeed
        );

        setModuleStates(moduleStates);
    }

    /**
     * Set desired states for all modules. Order: FL, FR, BL, BR
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Stop all modules.
     */
    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Set wheels to X formation (defensive stance - hard to push).
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Get current states of all modules.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    /**
     * Get current positions of all modules.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    /**
     * Get estimated robot pose on field.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset pose to specific position.
     */
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Get gyro rotation (negated to match WPILib coordinate system).
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-m_gyro.getYaw());
    }

    /**
     * Get heading in radians, wrapped to [-π, π].
     */
    public double getHeading() {
        return MathUtil.angleModulus(getRotation2d().getRadians());
    }

    /**
     * Zero the gyro - sets current direction as forward (0°).
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Get current robot-relative chassis speeds.
     */

    /**
     * Reset all module encoders to their absolute encoder readings.
     */
    public void resetEncoders() {
        m_frontLeft.resetToAbsolute();
        m_frontRight.resetToAbsolute();
        m_backLeft.resetToAbsolute();
        m_backRight.resetToAbsolute();
    }

    /**
     * Called every 20ms - updates pose estimate and publishes telemetry.
     */
    @Override
    public void periodic() {
        // Update pose estimate with latest sensor data
        m_poseEstimator.update(getRotation2d(), getModulePositions());
        
        // Update field visualization
        m_field.setRobotPose(getPose());

        // Publish telemetry to dashboard
        SmartDashboard.putNumber("Gyro Heading", Math.toDegrees(getHeading()));
        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        Logger.recordOutput("Robot Pose", getPose());
        SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getDegrees());
    }
}