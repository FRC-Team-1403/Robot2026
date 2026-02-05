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

public class SwerveModule {
    private final SparkMax m_driveMotor;
    private final SparkMax m_steerMotor;
    private final CANcoder m_absoluteEncoder;
    
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_steerEncoder;
    
    private final SparkClosedLoopController m_driveController;
    private final SparkClosedLoopController m_steerController;
    
    private final SimpleMotorFeedforward m_driveFeedforward;
    
    private final double m_absoluteEncoderOffset;
    
    private final String m_name;

    public SwerveModule(
        String name,
        int driveMotorID, 
        int steerMotorID, 
        int canCoderID,
        double absoluteEncoderOffset,
        boolean driveInverted
    ) {
        m_name = name;
        m_absoluteEncoderOffset = absoluteEncoderOffset;
        
        m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        m_steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        m_absoluteEncoder = new CANcoder(canCoderID);
        
        m_driveEncoder = m_driveMotor.getEncoder();
        m_steerEncoder = m_steerMotor.getEncoder();
        
        m_driveController = m_driveMotor.getClosedLoopController();
        m_steerController = m_steerMotor.getClosedLoopController();
        
        m_driveFeedforward = new SimpleMotorFeedforward(
            SwerveConstants.kSDrive,
            SwerveConstants.kVDrive,
            SwerveConstants.kADrive
        );
        
        configureEncoders();
        configureDriveMotor(driveInverted);
        configureSteerMotor();
    }
    
    private void configureEncoders() {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        magnetConfig.MagnetOffset = Units.radiansToRotations(m_absoluteEncoderOffset);
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5; // Equivalent to Signed_PlusMinusHalf (-0.5 to +0.5)
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
        
        // Set initial position after configuration
        m_steerEncoder.setPosition(getAbsoluteAngle());
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState currentState = getState();
        
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
        return MathUtil.angleModulus(Units.rotationsToRadians(
            m_absoluteEncoder.getAbsolutePosition().refresh().getValueAsDouble()));
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
}