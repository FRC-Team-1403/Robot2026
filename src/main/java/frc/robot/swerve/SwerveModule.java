package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
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
     * Calculates the base voltage needed to achieve a target velocity,
     * reducing the workload on the PID controller and improving response time.
     * 
     * Formula: V = kS + kV*velocity + kA*acceleration
     * - kS: Voltage to overcome static friction
     * - kV: Voltage per unit of velocity
     * - kA: Voltage per unit of acceleration
     */
    private final SimpleMotorFeedforward m_driveFeedforward;
    
    // ==================== Configuration ====================
    
    /** 
     * Offset value for the absolute encoder in radians.
     * This compensates for mechanical misalignment, ensuring that when the wheel
     * points straight forward, the encoder reads 0°.
     * 
     * Calibration process:
     * 1. Manually align wheel to point straight forward
     * 2. Read raw absolute encoder value
     * 3. Set offset = -rawValue
     * 4. Verify wheel angle reads 0° when pointing forward
     */
    private final double m_absoluteEncoderOffset;
    
    /** 
     * Human-readable name for this module (e.g., "Front Left").
     * Used for debugging and telemetry display.
     */
    private final String m_name;

    /**
     * Constructs a new SwerveModule.
     * 
     * Initialization sequence:
     * 1. Store configuration parameters
     * 2. Create motor controller and encoder objects
     * 3. Configure absolute encoder with offset and direction
     * 4. Configure drive motor with PID, limits, and conversions
     * 5. Configure steer motor with PID, wrapping, and conversions
     * 6. Sync steer encoder to absolute encoder position
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
            SwerveConstants.kSDrive,  // Static gain (overcome friction)
            SwerveConstants.kVDrive,  // Velocity gain
            SwerveConstants.kADrive   // Acceleration gain
        );
        
        // Configure all components
        configureEncoders();
        configureDriveMotor(driveInverted);
        configureSteerMotor();
    }
    
    /**
     * Configures the absolute encoder (CANcoder) with offset and settings.
     * 
     * Configuration includes:
     * - Magnet offset: Applies calibration offset to align 0° with forward
     * - Discontinuity point: Sets range to -0.5 to +0.5 rotations (-180° to +180°)
     * - Sensor direction: Counter-clockwise positive (standard math convention)
     * - Update frequency: 100 Hz for responsive readings
     * - Bus optimization: Reduces CAN traffic for non-critical signals
     * 
     * The absolute encoder provides the ground truth for wheel angle,
     * especially important after power cycling when relative encoders reset.
     */
    private void configureEncoders() {
        // Create magnet sensor configuration object
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        
        // Convert offset from radians to rotations and apply
        // This shifts the "zero" position to where we want forward to be
        magnetConfig.MagnetOffset = Units.radiansToRotations(m_absoluteEncoderOffset);
        
        // Set discontinuity point to 0.5 (equivalent to ±180°)
        // This means values wrap from +0.5 to -0.5, keeping angles bounded
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5;
        
        // Set sensor direction to counter-clockwise positive (standard convention)
        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        // Apply magnet configuration to overall CANcoder config
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor = magnetConfig;
        
        // Write configuration to the CANcoder hardware
        m_absoluteEncoder.getConfigurator().apply(canCoderConfig);
        
        // Set update frequency to 100 Hz for responsive readings
        m_absoluteEncoder.getAbsolutePosition().setUpdateFrequency(100);
        
        // Optimize CAN bus usage by reducing update rate of unused signals
        m_absoluteEncoder.optimizeBusUtilization();
    }
    
    /**
     * Configures the drive motor with all necessary settings.
     * 
     * Configuration includes:
     * - Motor inversion: Matches mechanical layout
     * - Idle mode: Brake mode to prevent coasting
     * - Current limit: Protects motor from overheating
     * - Voltage compensation: Ensures consistent performance across battery voltage
     * - Position conversion: Converts encoder rotations to meters
     * - Velocity conversion: Converts encoder RPM to meters per second
     * - PID gains: For velocity control
     * - Signal update rates: Determines how often encoder data is sent
     * 
     * The drive motor uses velocity control with feedforward for smooth,
     * accurate speed tracking.
     * 
     * @param inverted Whether to reverse the motor direction
     */
    private void configureDriveMotor(boolean inverted) {
        // Create configuration object for this SparkMax
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        
        // Set motor direction (depends on mechanical layout)
        driveConfig.inverted(inverted);
        
        // Set brake mode (motors actively resist motion when stopped)
        driveConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        
        // Set current limit to prevent motor damage and brownouts
        driveConfig.smartCurrentLimit(SwerveConstants.kDriveCurrentLimit);
        
        // Enable voltage compensation for consistent performance
        // Normalizes commands as if battery is always at 12V
        driveConfig.voltageCompensation(12.0);
        
        // Configure encoder conversion factors
        driveConfig.encoder
            // Convert encoder rotations to meters traveled
            // Factor accounts for wheel size and gear ratio
            .positionConversionFactor(SwerveConstants.kDrivePositionConversionFactor)
            // Convert encoder RPM to meters per second
            .velocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor);
        
        // Configure closed-loop (PID) control
        driveConfig.closedLoop
            // Use the built-in encoder as feedback
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID gains for velocity control
            .pid(SwerveConstants.kPDrive, SwerveConstants.kIDrive, SwerveConstants.kDDrive)
            // Limit output to ±100% duty cycle
            .outputRange(-1, 1);
        
        // Configure signal update rates (how often data is sent over CAN)
        driveConfig.signals
            // Update position every 20ms (50 Hz) - good balance of accuracy and CAN traffic
            .primaryEncoderPositionPeriodMs(20)
            // Update velocity every 20ms (50 Hz)
            .primaryEncoderVelocityPeriodMs(20);
        
        // Apply configuration to the hardware
        // ResetMode.kResetSafeParameters: Reset to safe defaults before applying
        // PersistMode.kPersistParameters: Save to flash memory (survives power cycle)
        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Configures the steer motor with all necessary settings.
     * 
     * Configuration includes:
     * - Motor inversion: False for standard CCW-positive convention
     * - Idle mode: Brake mode to hold position firmly
     * - Current limit: Protects motor and prevents brownouts
     * - Voltage compensation: Ensures consistent behavior
     * - Position conversion: Converts encoder rotations to radians
     * - Velocity conversion: Converts encoder RPM to radians per second
     * - PID gains: For angle control
     * - Position wrapping: Allows continuous rotation (e.g., -170° → 190°)
     * - Initial position: Synced to absolute encoder on startup
     * 
     * The steer motor uses position control with wrapping enabled,
     * allowing it to take the shortest path to any angle.
     */
    private void configureSteerMotor() {
        // Create configuration object
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        
        // Don't invert steer motor (standard CCW-positive)
        steerConfig.inverted(false);
        
        // Set brake mode (important for holding position)
        steerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        
        // Set current limit (steering needs less current than driving)
        steerConfig.smartCurrentLimit(SwerveConstants.kSteerCurrentLimit);
        
        // Enable voltage compensation
        steerConfig.voltageCompensation(12.0);
        
        // Configure encoder conversion factors
        steerConfig.encoder
            // Convert encoder rotations to radians
            // Factor accounts for steering gear ratio
            .positionConversionFactor(SwerveConstants.kSteerPositionConversionFactor)
            // Convert encoder RPM to radians per second
            .velocityConversionFactor(SwerveConstants.kSteerVelocityConversionFactor);
        
        // Configure closed-loop (PID) control
        steerConfig.closedLoop
            // Use built-in encoder as feedback
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID gains for angle control
            .pid(SwerveConstants.kPSteer, SwerveConstants.kISteer, SwerveConstants.kDSteer)
            // Limit output to ±100% duty cycle
            .outputRange(-1, 1)
            // Enable position wrapping for continuous rotation
            .positionWrappingEnabled(true)
            // Set wrapping range to ±π radians (±180°)
            // This allows the controller to wrap from -π to +π
            .positionWrappingInputRange(-Math.PI, Math.PI);
        
        // Configure signal update rates
        steerConfig.signals
            // Update position every 20ms (50 Hz)
            .primaryEncoderPositionPeriodMs(20);
        
        // Apply configuration to hardware
        m_steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Sync steer encoder to absolute encoder position
        // This ensures the relative encoder starts at the correct angle
        m_steerEncoder.setPosition(getAbsoluteAngle());
    }
    
    /**
     * Commands the module to a desired state (speed and angle).
     * 
     * This is the primary method called during operation. It performs:
     * 1. Gets the current state for optimization
     * 2. Optimizes the desired state (shortest rotation, possibly reverse drive)
     * 3. Commands steer motor to target angle using PID
     * 4. Calculates feedforward voltage for drive motor
     * 5. Commands drive motor to target velocity using PID + feedforward
     * 
     * Optimization example:
     * - Current angle: 0°, Desired angle: 270°
     * - Instead of rotating 270° CCW, it rotates 90° CW and reverses drive
     * - This is faster and reduces mechanical wear
     * 
     * @param desiredState Target state containing speed (m/s) and angle (Rotation2d)
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Get current state for optimization calculation
        SwerveModuleState currentState = getState();
        
        // Optimize the desired state
        // This may reverse the drive direction and adjust angle to minimize rotation
        // For example: instead of turning 270°, turn 90° and reverse drive
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentState.angle);
        
        // Extract optimized angle and speed
        double angleSetpoint = optimizedState.angle.getRadians();
        double velocitySetpoint = optimizedState.speedMetersPerSecond;
        
        // Command steer motor to target angle using position control
        // The PID controller will drive the motor to achieve this angle
        m_steerController.setSetpoint(angleSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        
        // Calculate feedforward voltage for drive motor
        // This predicts the voltage needed to achieve the target velocity
        // Reduces the work the PID controller needs to do
        double feedforward = m_driveFeedforward.calculate(velocitySetpoint);
        
        // Command drive motor to target velocity using velocity control + feedforward
        // The feedforward voltage is added to the PID output for better tracking
        m_driveController.setSetpoint(
            velocitySetpoint,                           // Target velocity (m/s)
            ControlType.kVelocity,                      // Velocity control mode
            ClosedLoopSlot.kSlot0,                      // Use PID slot 0
            feedforward,                                // Feedforward voltage
            SparkClosedLoopController.ArbFFUnits.kVoltage  // Units for feedforward
        );
    }
    
    /**
     * Gets the current state of this module.
     * 
     * The state contains:
     * - Velocity: Current wheel speed in meters per second (from drive encoder)
     * - Angle: Current wheel direction in radians (from steer encoder)
     * 
     * This is used for:
     * - State optimization (finding shortest path to desired state)
     * - Odometry (tracking robot movement)
     * - Debugging and telemetry
     * 
     * @return Current module state with velocity and angle
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(),                        // Current velocity (m/s)
            Rotation2d.fromRadians(m_steerEncoder.getPosition()) // Current angle (radians)
        );
    }
    
    /**
     * Gets the current position of this module.
     * 
     * The position contains:
     * - Distance: Total distance traveled in meters (from drive encoder)
     * - Angle: Current wheel direction in radians (from steer encoder)
     * 
     * This is used by the pose estimator to calculate how far the robot has moved.
     * Unlike velocity, distance accumulates over time and must be properly managed.
     * 
     * @return Current module position with distance and angle
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),                        // Total distance (meters)
            Rotation2d.fromRadians(m_steerEncoder.getPosition()) // Current angle (radians)
        );
    }
    
    /**
     * Reads the current angle from the absolute encoder.
     * 
     * This method:
     * 1. Refreshes the absolute position reading from the CANcoder
     * 2. Extracts the value as a double (in rotations)
     * 3. Converts rotations to radians
     * 4. Wraps the angle to the range [-π, π]
     * 
     * The absolute encoder provides the "ground truth" wheel angle,
     * which is critical after power cycling when relative encoders reset to zero.
     * 
     * @return Current absolute angle in radians, wrapped to [-π, π]
     */
    private double getAbsoluteAngle() {
        return MathUtil.angleModulus(                                     // Wrap to [-π, π]
            Units.rotationsToRadians(                                     // Convert to radians
                m_absoluteEncoder.getAbsolutePosition()                   // Get position signal
                    .refresh()                                            // Force fresh reading
                    .getValueAsDouble()));                                // Extract value
    }
    
    /**
     * Resets the steer encoder to match the absolute encoder.
     * 
     * This synchronizes the relative encoder (which may have drifted or reset)
     * with the absolute encoder (which knows the true angle).
     * 
     * This is typically called:
     * - During module initialization (in configureSteerMotor)
     * - When the driver presses a "reset encoders" button
     * - If the encoder readings seem incorrect
     * 
     * After this call, the relative encoder will report the same angle
     * as the absolute encoder.
     */
    public void resetToAbsolute() {
        m_steerEncoder.setPosition(getAbsoluteAngle());
    }
    
    /**
     * Stops both the drive and steer motors.
     * 
     * Sets the motor output to zero, causing both motors to stop.
     * In brake mode, the motors will actively resist motion.
     * In coast mode, the motors would spin freely.
     * 
     * This is called when:
     * - Commands end or are interrupted
     * - Emergency stop is triggered
     * - Robot is disabled
     */
    public void stop() {
        m_driveMotor.stopMotor();
        m_steerMotor.stopMotor();
    }
    
    /**
     * Gets the human-readable name of this module.
     * 
     * Used for debugging, logging, and telemetry display.
     * Examples: "Front Left", "Front Right", "Back Left", "Back Right"
     * 
     * @return Module name string
     */
    public String getName() {
        return m_name;
    }
}