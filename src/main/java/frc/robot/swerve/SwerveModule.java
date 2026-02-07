package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * SwerveModule class - represents a single swerve module (one corner of the robot).
 *
 * Each module consists of:
 * - Drive motor: Controls wheel speed (forward/backward motion)
 * - Steer motor: Controls wheel angle (direction of motion)
 * - Absolute encoder: Tracks true wheel angle, survives power cycles
 * - Relative encoders: Track distance traveled and current angle
 *
 * The module handles:
 * - Converting desired states (speed, angle) into motor commands
 * - Optimizing wheel rotations (never rotate more than 90°)
 * - Maintaining accurate position and velocity through encoders
 * - PID control for both speed and steering
 * - Feedforward control for smooth velocity tracking
 *
 * Hardware configuration:
 * - 2x REV SparkMax motor controllers (NEO motors)
 * - 1x CTRE CANcoder absolute encoder
 * - All communication via CAN bus
 */
public class SwerveModule {

    // ==================== Hardware Components ====================

    /**
     * Drive motor controller (SparkMax with NEO motor).
     * Responsible for spinning the wheel to achieve desired velocity.
     */
    private final SparkMax m_driveMotor;

    /**
     * Steer motor controller (SparkMax with NEO motor).
     * Responsible for rotating the module to achieve desired angle.
     */
    private final SparkMax m_steerMotor;

    /**
     * Absolute encoder (CANcoder) that tracks the true wheel angle.
     * This encoder remembers its position even after power cycling,
     * allowing the module to know its angle immediately on startup.
     */
    private final CANcoder m_absoluteEncoder;

    // ==================== Sensors ====================

    /**
     * Relative encoder built into the drive motor.
     * Tracks wheel rotations to calculate distance traveled and current velocity.
     * More accurate than absolute encoder for velocity measurements.
     */
    private final RelativeEncoder m_driveEncoder;

    /**
     * Relative encoder built into the steer motor.
     * Tracks steering angle changes for precise position control.
     * Reset to absolute encoder value on startup for consistency.
     */
    private final RelativeEncoder m_steerEncoder;

    // ==================== Controllers ====================

    /**
     * PID controller for the drive motor.
     * Controls wheel velocity to match desired speed using closed-loop feedback.
     */
    private final SparkClosedLoopController m_driveController;

    /**
     * PID controller for the steer motor.
     * Controls steering angle to match desired direction using closed-loop feedback.
     */
    private final SparkClosedLoopController m_steerController;

    /**
     * Feedforward controller for the drive motor.
     * Calculates the base voltage needed to achieve a target velocity.
     */
    private final SimpleMotorFeedforward m_driveFeedforward;

    // ==================== Configuration ====================

    /** Offset value for the absolute encoder in radians. */
    private final double m_absoluteEncoderOffset;

    /** Human-readable name for this module (e.g., "Front Left"). */
    private final String m_name;

    // EDIT (SysId): store last commanded drive volts as fallback for logging
    private double m_lastDriveVolts = 0.0;

    /**
     * Constructs a new SwerveModule.
     *
     * @param name Human-readable module name for debugging
     * @param driveMotorID CAN ID of the drive motor SparkMax
     * @param steerMotorID CAN ID of the steer motor SparkMax
     * @param canCoderID CAN ID of the absolute encoder (CANcoder)
     * @param absoluteEncoderOffset Offset in radians to calibrate absolute encoder
     * @param driveInverted Whether to invert the drive motor direction
     */
    public SwerveModule(
        String name,
        int driveMotorID,
        int steerMotorID,
        int canCoderID,
        double absoluteEncoderOffset,
        boolean driveInverted
    ) {
        // Store configuration
        m_name = name;
        m_absoluteEncoderOffset = absoluteEncoderOffset;

        // Initialize hardware objects
        m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        m_steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        m_absoluteEncoder = new CANcoder(canCoderID);

        // Get encoder objects from motor controllers
        m_driveEncoder = m_driveMotor.getEncoder();
        m_steerEncoder = m_steerMotor.getEncoder();

        // Get PID controller objects
        m_driveController = m_driveMotor.getClosedLoopController();
        m_steerController = m_steerMotor.getClosedLoopController();

        // Initialize feedforward controller with constants
        m_driveFeedforward = new SimpleMotorFeedforward(
            SwerveConstants.kSDrive,
            SwerveConstants.kVDrive,
            SwerveConstants.kADrive
        );

        // Configure all components
        configureEncoders();
        configureDriveMotor(driveInverted);
        configureSteerMotor();
    }

    private void configureEncoders() {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();

        magnetConfig.MagnetOffset = Units.radiansToRotations(m_absoluteEncoderOffset);
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5;
        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor = magnetConfig;

        m_absoluteEncoder.getConfigurator().apply(canCoderConfig);

        m_absoluteEncoder.getAbsolutePosition().setUpdateFrequency(100);
        m_absoluteEncoder.optimizeBusUtilization();
    }

    private void configureDriveMotor(boolean inverted) {
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig.inverted(inverted);
        driveConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        driveConfig.smartCurrentLimit(SwerveConstants.kDriveCurrentLimit);
        driveConfig.voltageCompensation(12.0);

        driveConfig.encoder
            .positionConversionFactor(SwerveConstants.kDrivePositionConversionFactor)
            .velocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor);

        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveConstants.kPDrive, SwerveConstants.kIDrive, SwerveConstants.kDDrive)
            .outputRange(-1, 1);

        driveConfig.signals
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(20);

        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureSteerMotor() {
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig.inverted(false);
        steerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        steerConfig.smartCurrentLimit(SwerveConstants.kSteerCurrentLimit);
        steerConfig.voltageCompensation(12.0);

        steerConfig.encoder
            .positionConversionFactor(SwerveConstants.kSteerPositionConversionFactor)
            .velocityConversionFactor(SwerveConstants.kSteerVelocityConversionFactor);

        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveConstants.kPSteer, SwerveConstants.kISteer, SwerveConstants.kDSteer)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);

        steerConfig.signals
            .primaryEncoderPositionPeriodMs(20);

        m_steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_steerEncoder.setPosition(getAbsoluteAngle());
    }

    /**
     * Commands the module to a desired state (speed and angle).
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState currentState = getState();

        @SuppressWarnings("all")
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentState.angle);

        double angleSetpoint = optimizedState.angle.getRadians();
        double velocitySetpoint = optimizedState.speedMetersPerSecond;

        m_steerController.setSetpoint(angleSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        double feedforward = m_driveFeedforward.calculate(velocitySetpoint);

        m_driveController.setSetpoint(
            velocitySetpoint,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedforward,
            SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(),
            Rotation2d.fromRadians(m_steerEncoder.getPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            Rotation2d.fromRadians(m_steerEncoder.getPosition())
        );
    }

    private double getAbsoluteAngle() {
        return MathUtil.angleModulus(
            Units.rotationsToRadians(
                m_absoluteEncoder.getAbsolutePosition()
                    .refresh()
                    .getValueAsDouble()));
    }

    public void resetToAbsolute() {
        m_steerEncoder.setPosition(getAbsoluteAngle());
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_steerMotor.stopMotor();
    }

    public String getName() {
        return m_name;
    }

    // =========================================================
    // EDIT (SysId): Methods used by SwerveDrive SysIdRoutine
    // =========================================================

    /**
     * EDIT (SysId): Hold the steering angle without commanding the drive loop.
     */
    public void holdAngle(Rotation2d angle) {
        m_steerController.setSetpoint(
            angle.getRadians(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }

    /**
     * EDIT (SysId): Apply open-loop voltage to the DRIVE motor (not velocity control).
     */
    public void setDriveVoltage(double volts) {
        m_lastDriveVolts = volts;
        m_driveMotor.setVoltage(volts);
    }

    /**
     * EDIT (SysId): Drive distance in meters.
     */
    public double getDriveDistanceMeters() {
        return m_driveEncoder.getPosition();
    }

    /**
     * EDIT (SysId): Drive speed in meters per second.
     */
    public double getDriveSpeedMetersPerSec() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * EDIT (SysId): Applied drive volts for logging.
     */
    public double getAppliedDriveVolts() {
        try {
            return m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
        } catch (Exception e) {
            return m_lastDriveVolts;
        }
    }
}
