package team1403.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.util.CustomPositionControlLoop;

import org.littletonrobotics.junction.Logger;

public class IntakeWrist extends SubsystemBase {
  private final TalonFX m_motor;
  private final CANcoder m_encoder;
  private final CustomPositionControlLoop m_customController;
  private final DutyCycleOut m_wristDutyCycleRequest;
  private double currentAngle;
  private double setpoint;

  public IntakeWrist() {
    m_motor = new TalonFX(Constants.IntakeWrist.kWristMotorID);

    m_wristDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor.getConfigurator().apply(wristMotorConfig);

    m_encoder = new CANcoder(Constants.IntakeWrist.kEncoderID);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    m_encoder.getConfigurator().apply(encoderConfig);

    resetEncoder();

    m_customController =
        new CustomPositionControlLoop(
            Constants.IntakeWrist.kGain,
            Constants.IntakeWrist.kToleranceDegrees,
            Constants.IntakeWrist.kRampUpTime,
            Constants.IntakeWrist.kRampDownTime,
            Constants.IntakeWrist.kUnitsPerRampTime,
            Constants.IntakeWrist.kMaxSpeed,
            Constants.IntakeWrist.kMinSpeed,
            Constants.IntakeWrist.kLoopTime);

    currentAngle = getWristAngle();
    setpoint = currentAngle;
  }

  public double getWristAngle() {
    double rotations = m_encoder.getPosition().getValueAsDouble();
    double degrees = rotations / Constants.IntakeWrist.kGearRatio * 360.0;
    return degrees;
  }

  public void setSetpoint(double degrees) {
    double correctedDegrees =
        MathUtil.clamp(
            degrees,
            Constants.IntakeWrist.kMinAngleDegrees,
            Constants.IntakeWrist.kMaxAngleDegrees);
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
    m_wristDutyCycleRequest.Output = 0.0;
    m_motor.setControl(m_wristDutyCycleRequest);
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
    m_wristDutyCycleRequest.Output = output;
    m_motor.setControl(m_wristDutyCycleRequest);
  }

  @Override
  public void periodic() {
    currentAngle = getWristAngle();

    double smallestError = getError(setpoint, currentAngle);
    double motorOutput = m_customController.calculate(smallestError, currentAngle, setpoint);

    setMotorOutput(motorOutput / 100.0);

    if (currentAngle >= Constants.IntakeWrist.kMaxAngleDegrees && motorOutput > 0) {
      motorOutput = 0;
    } else if (currentAngle <= Constants.IntakeWrist.kMinAngleDegrees && motorOutput < 0) {
      motorOutput = 0;
    }

    Logger.recordOutput("IntakeWrist/Intake Wrist Current Angle", currentAngle);
    Logger.recordOutput("IntakeWrist/Setpoint", setpoint);
    Logger.recordOutput("IntakeWrist/At Setpoint", atSetpoint());
    Logger.recordOutput("IntakeWrist/Motor Output", motorOutput);
    Logger.recordOutput("IntakeWrist/P Value", m_customController.getP());
    Logger.recordOutput("IntakeWrist/Position Error", smallestError);
    Logger.recordOutput(
        "IntakeWrist/Encoder Rotations", m_encoder.getPosition().getValueAsDouble());
  }
}
