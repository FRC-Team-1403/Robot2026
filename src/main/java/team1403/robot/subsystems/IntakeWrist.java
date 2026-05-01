package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class IntakeWrist extends SubsystemBase {
  private final TalonFX m_intakeWristMotor;
  private final CANcoder m_intakeWristEncoder;
  private final PositionVoltage m_positionVoltageRequest;
  private final DutyCycleOut m_dutyCycleRequest;
  private final NeutralOut m_neutralRequest;

  private double currentPosition;
  private double setpoint;

  public IntakeWrist() {
    m_intakeWristMotor = new TalonFX(Constants.IntakeWrist.kWristMotorID, "Bus 2");
    m_positionVoltageRequest = new PositionVoltage(0);
    m_dutyCycleRequest = new DutyCycleOut(0);
    m_neutralRequest = new NeutralOut();

    m_positionVoltageRequest.EnableFOC = true;
    m_dutyCycleRequest.EnableFOC = true;
    
    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristMotorConfig.Feedback.FeedbackRemoteSensorID = Constants.IntakeWrist.kEncoderID;
    wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    Slot0Configs wristPIDConfigs = new Slot0Configs();
    wristPIDConfigs.kP = Constants.IntakeWrist.kP;
    wristPIDConfigs.kI = Constants.IntakeWrist.kI;
    wristPIDConfigs.kD = Constants.IntakeWrist.kD;
    wristPIDConfigs.kS = Constants.IntakeWrist.kS;
    wristPIDConfigs.kV = Constants.IntakeWrist.kV;
    wristPIDConfigs.kA = Constants.IntakeWrist.kA;
    wristPIDConfigs.kG = Constants.IntakeWrist.kG;
    wristPIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    wristPIDConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    m_intakeWristMotor.getConfigurator().apply(wristMotorConfig);
    m_intakeWristMotor.getConfigurator().apply(wristPIDConfigs);

    m_intakeWristEncoder = new CANcoder(Constants.IntakeWrist.kEncoderID,"Bus 2");

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.IntakeWrist.kMagnetOffset;

    m_intakeWristEncoder.getConfigurator().apply(encoderConfig);

    currentPosition = getAbsolutePosition();
    setpoint = currentPosition;
    m_intakeWristEncoder.setPosition(m_intakeWristEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getAbsolutePosition() {
    return m_intakeWristEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public void setSetpoint(double rotations) {
    double correctedRotations =
        MathUtil.clamp(
            rotations,
            Constants.IntakeWrist.kMinRotations,
            Constants.IntakeWrist.kMaxRotations);
    setpoint = correctedRotations;
    m_intakeWristMotor.setControl(m_positionVoltageRequest.withPosition(setpoint));
  }

  public void setPower(double power) {
    m_dutyCycleRequest.Output = power;
    m_intakeWristMotor.setControl(m_dutyCycleRequest);
  }

  public double getSetpoint() {
    return setpoint;
  }

  public boolean atSetpoint() {
    return Math.abs(setpoint - currentPosition) <= Constants.IntakeWrist.kToleranceRotations;
  }

  public void adjustSetpoint(double rotations) {
    setSetpoint(setpoint + rotations);
  }

  public void stopMotor() {
    m_intakeWristMotor.setControl(m_neutralRequest);
  }

  public void resetEncoder() {
    m_intakeWristEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    currentPosition = getAbsolutePosition();

    Logger.recordOutput("IntakeWrist/Wrist Current Position", currentPosition);
    // Logger.recordOutput("IntakeWrist/Setpoint", setpoint);
    // Logger.recordOutput("IntakeWrist/At Setpoint", atSetpoint());
    // Logger.recordOutput("IntakeWrist/Encoder Rotations", m_intakeWristEncoder.getPosition().getValueAsDouble());
    // Logger.recordOutput("IntakeWrist/StatorCurrent", m_intakeWristMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("IntakeWrist/SupplyCurrent", m_intakeWristMotor.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput("IntakeWrist/Device Temperature", m_intakeWristMotor.getDeviceTemp().getValueAsDouble());
  }
}