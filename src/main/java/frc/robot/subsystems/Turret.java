package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

public class Turret extends SubsystemBase {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;
    private final CustomPositionControlLoop m_customController;
    private final DutyCycleOut m_turretDutyCycleRequest;
    private double currentAngle;
    private double setpoint;

    public Turret() {
        m_motor = new TalonFX(Constants.Turret.kTurretMotorID);

        m_turretDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        turretMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motor.getConfigurator().apply(turretMotorConfig);

        m_encoder = new CANcoder(Constants.Turret.kEncoderID);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_encoder.getConfigurator().apply(encoderConfig);

        resetEncoder();

        m_customController = new CustomPositionControlLoop(
                Constants.Turret.kGain,
                Constants.Turret.kToleranceDegrees,
                Constants.Turret.kRampUpTime,
                Constants.Turret.kRampDownTime,
                Constants.Turret.kUnitsPerRampTime,
                Constants.Turret.kMaxSpeed,
                Constants.Turret.kMinSpeed,
                Constants.Turret.kLoopTime);

        currentAngle = getTurretAngle();
        setpoint = currentAngle;
    }

    public double getTurretAngle() {
        double rotations = m_encoder.getPosition().getValueAsDouble();
        double degrees = rotations / Constants.Turret.kGearRatio * 360.0;
        return degrees;
    }

    public void setSetpoint(double degrees) {
        double correctedDegrees = MathUtil.clamp(degrees, Constants.Turret.kMinAngleDegrees,
                Constants.Turret.kMaxAngleDegrees);
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
        m_turretDutyCycleRequest.Output = 0.0;
        m_motor.setControl(m_turretDutyCycleRequest);
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
        m_turretDutyCycleRequest.Output = output;
        m_motor.setControl(m_turretDutyCycleRequest);
    }

    @Override
    public void periodic() {
        currentAngle = getTurretAngle();

        double smallestError = getError(setpoint, currentAngle);
        double motorOutput = m_customController.calculate(smallestError, currentAngle, setpoint);

        setMotorOutput(motorOutput / 100.0);

        if (currentAngle >= Constants.Turret.kMaxAngleDegrees && motorOutput > 0) {
            motorOutput = 0;
        } else if (currentAngle <= Constants.Turret.kMinAngleDegrees && motorOutput < 0) {
            motorOutput = 0;
        }

        Logger.recordOutput("Turret/Turret Current Angle", currentAngle);
        Logger.recordOutput("Turret/Setpoint", setpoint);
        Logger.recordOutput("Turret/At Setpoint", atSetpoint());
        Logger.recordOutput("Turret/Motor Output", motorOutput);
        Logger.recordOutput("Turret/P Value", m_customController.getP());
        Logger.recordOutput("Turret/Position Error", smallestError);
        Logger.recordOutput("Turret/Encoder Rotations", m_encoder.getPosition().getValueAsDouble());
    }
}