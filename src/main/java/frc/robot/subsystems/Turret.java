package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

public class Turret extends SubsystemBase {
    private final TalonSRX m_motor;
    private final CANcoder m_encoder;
    private final CustomPositionControlLoop m_customController;
    private double currentAngle;
    private double setpoint;

    public Turret() {
        m_motor = new TalonSRX(Constants.TurretConstants.kTurretMotorID);
        m_motor.setInverted(Constants.TurretConstants.kMotorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);

        m_encoder = new CANcoder(0);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_encoder.getConfigurator().apply(config);

        resetEncoder();

        m_customController = new CustomPositionControlLoop(
                Constants.TurretConstants.kGain,
                Constants.TurretConstants.kToleranceDegrees,
                Constants.TurretConstants.kRampUpTime,
                Constants.TurretConstants.kRampDownTime,
                Constants.TurretConstants.kUnitsPerRampTime,
                Constants.TurretConstants.kMaxSpeed,
                Constants.TurretConstants.kMinSpeed,
                Constants.TurretConstants.kLoopTime);

        currentAngle = getTurretAngle();
        setpoint = currentAngle;
    }

    // Make sure this returns +ve angles for CCW rotation
    public double getTurretAngle() {
        double rotations = m_encoder.getPosition().getValueAsDouble();
        double degrees = rotations / Constants.TurretConstants.kGearRatio * 360.0;
        return degrees;
    }

    public void setSetpoint(double degrees) {
        double correctedDegrees = MathUtil.clamp(degrees, Constants.TurretConstants.kMinAngleDegrees,
                Constants.TurretConstants.kMaxAngleDegrees);
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
        m_motor.set(ControlMode.PercentOutput, 0.0);
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
        m_motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void periodic() {
        currentAngle = getTurretAngle();

        double smallestError = getError(setpoint, currentAngle);
        double motorOutput = m_customController.calculate(smallestError, currentAngle, setpoint);

        setMotorOutput(motorOutput / 100.0);

        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Setpoint", setpoint);
        SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Motor Output", motorOutput);
        SmartDashboard.putNumber("Turret/P Value", m_customController.getP());
        SmartDashboard.putNumber("Turret/Position Error", smallestError);
        SmartDashboard.putNumber("Turret/Encoder Rotations", m_encoder.getPosition().getValueAsDouble());
    }
}