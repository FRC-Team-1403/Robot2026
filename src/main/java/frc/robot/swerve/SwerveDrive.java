package frc.robot.swerve;

import com.studica.frc.AHRS;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d; // KEEP only one Rotation2d import (you had duplicates)
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; // EDIT: added for SysId command access
import edu.wpi.first.wpilibj2.command.SubsystemBase; // KEEP only one SubsystemBase import (you had duplicates)
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; // EDIT: added SysIdRoutine import

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
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

    // EDIT: added module array for looping (SysId + helpers)
    private final SwerveModule[] m_modules;

    // Tracking
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field;

    // EDIT: SysId fields
    private final SysIdRoutine m_sysId;
    private static final Rotation2d kSysIdAngle = Rotation2d.fromDegrees(0.0);

    public SwerveDrive() {

        // IMPORTANT: you had "private final SwerveModule[] modules;" inside the constructor.
        // EDIT: removed that (illegal Java). We use the class field m_modules instead.

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

        // EDIT: create array after modules exist
        m_modules = new SwerveModule[] { m_frontLeft, m_frontRight, m_backLeft, m_backRight };

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

        // EDIT: SysId routine creation (drive translation characterization)
        m_sysId = new SysIdRoutine(
            new SysIdRoutine.Config(), // defaults are fine to start
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    // EDIT: lock azimuth so wheels point straight during the test
                    lockAllAzimuth(kSysIdAngle);
                    // EDIT: apply the same drive voltage to all modules
                    setAllDriveVoltage(voltage.in(Volts));
                },
                (log) -> {
                    // EDIT: log average position/velocity/voltage across modules
                    log.motor("swerve-drive")
                        .voltage(Volts.of(getAverageAppliedDriveVolts()))
                        .linearPosition(Meters.of(getAverageDriveDistanceMeters()))
                        .linearVelocity(MetersPerSecond.of(getAverageDriveSpeedMps()));
                },
                this
            )
        );

        // Zero gyro after 1 second delay (allows gyro to initialize)
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();
    }

    /**
     * Configure PathPlanner for autonomous path following.
     */
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

    // =========================
    // EDIT: SysId public commands (bind these to buttons)
    // =========================

    public Command sysIdQuasistaticForward() {
        return m_sysId.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return m_sysId.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return m_sysId.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return m_sysId.dynamic(SysIdRoutine.Direction.kReverse);
    }

    // =========================
    // EDIT: SysId helper methods
    // =========================

    private void lockAllAzimuth(Rotation2d angle) {
        for (SwerveModule m : m_modules) {
            // EDIT: you must implement holdAngle(angle) in SwerveModule
            m.holdAngle(angle);
        }
    }

    private void setAllDriveVoltage(double volts) {
        for (SwerveModule m : m_modules) {
            // EDIT: you must implement setDriveVoltage(volts) in SwerveModule (open-loop voltage)
            m.setDriveVoltage(volts);
        }
    }

    private double getAverageDriveDistanceMeters() {
        double sum = 0.0;
        for (SwerveModule m : m_modules) {
            // EDIT: you must implement getDriveDistanceMeters() in SwerveModule
            sum += m.getDriveDistanceMeters();
        }
        return sum / m_modules.length;
    }

    private double getAverageDriveSpeedMps() {
        double sum = 0.0;
        for (SwerveModule m : m_modules) {
            // EDIT: you must implement getDriveSpeedMetersPerSec() in SwerveModule
            sum += m.getDriveSpeedMetersPerSec();
        }
        return sum / m_modules.length;
    }

    private double getAverageAppliedDriveVolts() {
        double sum = 0.0;
        for (SwerveModule m : m_modules) {
            // EDIT: you must implement getAppliedDriveVolts() in SwerveModule
            sum += m.getAppliedDriveVolts();
        }
        return sum / m_modules.length;
    }

    /**
     * Drive with X/Y speeds and rotation. Can be field-relative or robot-relative.
     */
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

    /**
     * Drive using robot-relative chassis speeds.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] moduleStates =
            SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

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
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

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
        m_poseEstimator.update(getRotation2d(), getModulePositions());
        m_field.setRobotPose(getPose());

        SmartDashboard.putNumber("Gyro Heading", Math.toDegrees(getHeading()));
        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        Logger.recordOutput("Robot Pose", getPose());
        SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getDegrees());
    }
}
