package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

public class ShooterHood extends SubsystemBase {
    private final TalonFX m_hoodMotor;
    private final CANcoder m_encoder;
    private final CustomPositionControlLoop m_customController;
    private double currentAngle;
    private double setpoint;

    public ShooterHood() {
        m_hoodMotor = new TalonFX(Constants.ShooterHood.kHoodMotorID);
        m_encoder = new CANcoder(0);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_encoder.getConfigurator().apply(config);

        resetEncoder();

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

    public double getHoodAngle() {
        double degrees = m_encoder.getPosition().getValueAsDouble();
        return degrees;
    }

    public void setSetpoint(double degrees) {
        double correctedDegrees = MathUtil.clamp(degrees, Constants.ShooterHood.kMinAngleDegrees,
                Constants.ShooterHood.kMaxAngleDegrees);
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
        m_hoodMotor.set(0.0);
        m_customController.reset();
    }

    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    private double getError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        return error;
    }

    private void setMotorOutput(double output) {
        m_hoodMotor.set(output);
    }

    @Override
    public void periodic() {
        currentAngle = getHoodAngle();

        double smallestError = getError(setpoint, currentAngle);
        double motorOutput = m_customController.calculate(smallestError, currentAngle, setpoint);

        if (currentAngle >= Constants.ShooterHood.kMaxAngleDegrees && motorOutput > 0) {
            motorOutput = 0; // At max limit, don't move more positive
        } else if (currentAngle <= Constants.ShooterHood.kMinAngleDegrees && motorOutput < 0) {
            motorOutput = 0; // At min limit, don't move more negative
        }
        
        setMotorOutput(motorOutput / 100.0);

        SmartDashboard.putNumber("Hood/Current Angle", currentAngle);
        SmartDashboard.putNumber("Hood/Setpoint", setpoint);
        SmartDashboard.putBoolean("Hood/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Hood/Motor Output", motorOutput);
        SmartDashboard.putNumber("Hood/P Value", m_customController.getP());
        SmartDashboard.putNumber("Hood/Position Error", smallestError);
        SmartDashboard.putNumber("Hood/Encoder Rotations", m_encoder.getPosition().getValueAsDouble());
    }

    
}