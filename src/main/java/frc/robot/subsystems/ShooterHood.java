package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

public class ShooterHood extends SubsystemBase {
    private final TalonFX m_hoodMotor;
    private final CANcoder m_encoder;
    private ArmFeedforward m_hoodFeedforward;
    private final DutyCycleOut m_dutyCycleRequest;
    private final NeutralOut m_neutralRequest;
    private final CustomPositionControlLoop m_customController;
    private double currentAngle;
    private double setpoint;

    public ShooterHood() {
        m_hoodMotor = new TalonFX(Constants.ShooterHood.kHoodMotorID,"Bus 1");
        m_encoder = new CANcoder(Constants.ShooterHood.kEncoderID,"Bus 1");
        m_dutyCycleRequest = new DutyCycleOut(0);
        m_neutralRequest = new NeutralOut();
        m_hoodFeedforward = new ArmFeedforward(Constants.ShooterHood.kS, Constants.ShooterHood.kG, Constants.ShooterHood.kV, Constants.ShooterHood.kA);
        
        TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_hoodMotor.getConfigurator().apply(hoodMotorConfig);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        config.MagnetSensor.MagnetOffset = 0;
        m_encoder.getConfigurator().apply(config);

        double absoluteRotations = getAbsolutePosition();
        double hoodRotations = absoluteRotations * Constants.ShooterHood.kGearRatioEncoder;
        m_hoodMotor.setPosition(hoodRotations);

        m_customController = new CustomPositionControlLoop(
                Constants.ShooterHood.kGain,
                Constants.ShooterHood.kToleranceDegrees,
                Constants.ShooterHood.kRampUpTime,
                Constants.ShooterHood.kRampDownTime,
                Constants.ShooterHood.kUnitsPerRampTime,
                Constants.ShooterHood.kMaxSpeed,
                Constants.ShooterHood.kMinSpeed,
                Constants.ShooterHood.kLoopTime);

        currentAngle = getHoodAngle();
        setpoint = currentAngle;
    }

    public double getAbsolutePosition(){
        return (m_encoder.getAbsolutePosition().getValueAsDouble()-0.794921875)*-1;
    }
    
    public double getHoodAngle() {
        double motorRotations = m_hoodMotor.getPosition().getValueAsDouble();
        double hoodRotations = motorRotations / Constants.ShooterHood.kGearRatioHoodAngleRatio;
        return Units.rotationsToDegrees(hoodRotations);
    }

    public void setSetpoint(double degrees) {
        double correctedDegrees = MathUtil.clamp(degrees, Constants.ShooterHood.kMinAngleDegrees, Constants.ShooterHood.kMaxAngleDegrees);
        setpoint = correctedDegrees;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return m_customController.isAtPosition();
    }

    public void adjustSetpoint(double degrees) {
        setSetpoint(setpoint + degrees);
    }

    public void stopMotor() {
        m_hoodMotor.setControl(m_neutralRequest);
        m_customController.reset();
    }

    private double getError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        return error;
    }

    private void setMotorOutput(double output) {
        m_dutyCycleRequest.Output = output;
        m_hoodMotor.setControl(m_dutyCycleRequest);
    }

    @Override
    public void periodic() {
        currentAngle = getHoodAngle();
        double smallestError = getError(setpoint, currentAngle);
        double controlLoop = m_customController.calculate(smallestError, currentAngle, setpoint);
        double ff = m_hoodFeedforward.calculate(Units.degreesToRadians(currentAngle), 0);
        double motorOutput = ff + controlLoop;

        if (currentAngle >= Constants.ShooterHood.kMaxAngleDegrees && motorOutput > 0) {
            motorOutput = 0;
        } else if (currentAngle <= Constants.ShooterHood.kMinAngleDegrees && motorOutput < 0) {
            motorOutput = 0;
        }

        setMotorOutput(motorOutput / 100.0);

        SmartDashboard.putNumber("Hood/Current Angle", currentAngle);
        SmartDashboard.putNumber("Hood/Absolute", getAbsolutePosition());
        SmartDashboard.putNumber("Hood/Setpoint", setpoint);
        SmartDashboard.putBoolean("Hood/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Hood/Motor Output", motorOutput);
        SmartDashboard.putNumber("Hood/P Value", m_customController.getP());
        SmartDashboard.putNumber("Hood/Position Error", smallestError);
        SmartDashboard.putNumber("Hood/Relative", m_hoodMotor.getPosition().getValueAsDouble());
    }
}