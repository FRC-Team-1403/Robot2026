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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.util.CustomPositionControlLoop;

public class IntakeWrist extends SubsystemBase {
  private final TalonFX m_intakeWristMotor;
  private final CANcoder m_intakeWristEncoder;
  private final CustomPositionControlLoop m_customController;
  private final DutyCycleOut m_wristDutyCycleRequest;
  private ArmFeedforward m_wristff;

  private double currentAngle;
  private double setpoint;

  public IntakeWrist() {
    m_intakeWristMotor = new TalonFX(Constants.IntakeWrist.kWristMotorID,"Bus 2");
    m_wristDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    m_wristff =
        new ArmFeedforward(
            Constants.IntakeWrist.kS,
            Constants.IntakeWrist.kG,
            Constants.IntakeWrist.kV,
            Constants.IntakeWrist.kA);

    m_intakeWristMotor.getConfigurator().apply(wristMotorConfig);

    m_intakeWristEncoder = new CANcoder(Constants.IntakeWrist.kEncoderID);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.IntakeWrist.kMagnetOffset;

    m_intakeWristEncoder.getConfigurator().apply(encoderConfig);

    double absoluteRotations = getAbsolutePosition();
    double wristRotations = absoluteRotations * Constants.IntakeWrist.kGearRatioEncoder;
    m_intakeWristMotor.setPosition(wristRotations);

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
    return m_customController.isAtPosition();
  }

  public void adjustSetpoint(double degrees) {
    setSetpoint(setpoint + degrees);
  }

  public void stopMotor() {
    m_wristDutyCycleRequest.Output = 0.0;
    m_intakeWristMotor.setControl(m_wristDutyCycleRequest);
    m_customController.reset();
  }

  public void resetEncoder() {
    m_intakeWristEncoder.setPosition(0.0);
  }

  private double getError(double targetAngle, double currentAngle) {
    double error = targetAngle - currentAngle;
    return error;
  }

  public void setMotorOutput(double output) {
    m_wristDutyCycleRequest.Output = output;
    m_intakeWristMotor.setControl(m_wristDutyCycleRequest);
  }

  @Override
  public void periodic() {
    currentAngle = getWristAngle();

    double smallestError = getError(setpoint, currentAngle);
    double controlloop = m_customController.calculate(smallestError, currentAngle, setpoint);
    double ff = m_wristff.calculate(Units.degreesToRadians(currentAngle), 0);
    double motorOutput = ff+controlloop;

    setMotorOutput(motorOutput / 100.0);

    if (currentAngle >= Constants.IntakeWrist.kMaxAngleDegrees && motorOutput > 0) {
      motorOutput = 0;
    } else if (currentAngle <= Constants.IntakeWrist.kMinAngleDegrees && motorOutput < 0) {
      motorOutput = 0;
    }

    Logger.recordOutput("IntakeWrist/Intake Wrist Current Angle", currentAngle);
    Logger.recordOutput("IntakeWrist/Setpoint", setpoint);
    Logger.recordOutput("IntakeWrist/At Setpoint", atSetpoint());
    //Logger.recordOutput("IntakeWrist/Motor Output", motorOutput);
    Logger.recordOutput("IntakeWrist/P Value", m_customController.getP());
    //Logger.recordOutput("IntakeWrist/Position Error", smallestError);
    Logger.recordOutput("IntakeWrist/Encoder Rotations", m_intakeWristEncoder.getPosition().getValueAsDouble());
    Logger.recordOutput("IntakeWrist/StatorCurrent", m_intakeWristMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("IntakeWrist/SupplyCurrent", m_intakeWristMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("IntakeWrist/Device Temperature", m_intakeWristMotor.getDeviceTemp().getValueAsDouble());

  }
}
