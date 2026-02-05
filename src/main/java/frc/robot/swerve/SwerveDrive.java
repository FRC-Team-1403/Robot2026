package frc.robot.swerve;

import com.studica.frc.AHRS;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final AHRS m_gyro;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field;

    public SwerveDrive() {

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

        m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);   
        m_poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.kDriveKinematics,
            getRotation2d(),
            getModulePositions(),
            new Pose2d()
        );

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        configurePathPlanner();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
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

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, getRotation2d()
            );
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        driveRobotRelative(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] moduleStates =
            SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            SwerveConstants.kMaxSpeedMetersPerSecond
        );

        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-m_gyro.getYaw());
    }

    public double getHeading() {
        return MathUtil.angleModulus(getRotation2d().getRadians());
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void resetEncoders() {
        m_frontLeft.resetToAbsolute();
        m_frontRight.resetToAbsolute();
        m_backLeft.resetToAbsolute();
        m_backRight.resetToAbsolute();
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(getRotation2d(), getModulePositions());
        m_field.setRobotPose(getPose());

        SmartDashboard.putNumber("Gyro Heading", Math.toDegrees(getHeading()));
        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getDegrees());
    }
}
