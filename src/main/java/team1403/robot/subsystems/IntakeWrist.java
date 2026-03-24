package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class IntakeWrist extends SubsystemBase {
  private final TalonFX m_intakeWristMotor;
  private final CANcoder m_intakeWristEncoder;
  private final DutyCycleOut m_wristDutyCycleRequest;

  private double currentAngle;
  private double setpoint;

  public IntakeWrist() {
    m_intakeWristMotor = new TalonFX(Constants.IntakeWrist.kWristMotorID, "Bus 2");
    m_wristDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    m_intakeWristMotor.getConfigurator().apply(wristMotorConfig);

    m_intakeWristEncoder = new CANcoder(Constants.IntakeWrist.kEncoderID);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.IntakeWrist.kMagnetOffset;

    m_intakeWristEncoder.getConfigurator().apply(encoderConfig);

    double absoluteRotations = getAbsolutePosition();
    double wristRotations = absoluteRotations * Constants.IntakeWrist.kGearRatioEncoder;
    m_intakeWristMotor.setPosition(wristRotations);

    currentAngle = getWristAngle();
    setpoint = currentAngle;
  }

  public double getAbsolutePosition() {
    return (m_intakeWristEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getWristAngle() {
    double motorRotations = m_intakeWristMotor.getPosition().getValueAsDouble();
    double hoodRotations = motorRotations / Constants.IntakeWrist.kGearRatioWristAngleRatio;
    return Units.rotationsToDegrees(hoodRotations);
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
    return Math.abs(setpoint - currentAngle) <= Constants.IntakeWrist.kToleranceDegrees;
  }

  public void adjustSetpoint(double degrees) {
    setSetpoint(setpoint + degrees);
  }

  public void stopMotor() {
    m_wristDutyCycleRequest.Output = 0.0;
    m_intakeWristMotor.setControl(m_wristDutyCycleRequest);
  }

  public void resetEncoder() {
    m_intakeWristEncoder.setPosition(0.0);
  }

  public void setMotorOutput(double output) {
    m_wristDutyCycleRequest.Output = output;
    m_intakeWristMotor.setControl(m_wristDutyCycleRequest);
  }

  @Override
  public void periodic() {
  
    currentAngle = getWristAngle();

    Logger.recordOutput("IntakeWrist/Intake Wrist Current Angle", currentAngle);
    Logger.recordOutput("IntakeWrist/Setpoint", setpoint);
    Logger.recordOutput("IntakeWrist/At Setpoint", atSetpoint());
    Logger.recordOutput("IntakeWrist/Encoder Rotations", m_intakeWristEncoder.getPosition().getValueAsDouble());
    Logger.recordOutput("IntakeWrist/StatorCurrent", m_intakeWristMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("IntakeWrist/SupplyCurrent", m_intakeWristMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("IntakeWrist/Device Temperature", m_intakeWristMotor.getDeviceTemp().getValueAsDouble());
  }
}