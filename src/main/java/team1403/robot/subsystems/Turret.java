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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.util.Blackbox;
import team1403.robot.util.CustomPositionControlLoop;

public class Turret extends SubsystemBase {
  private final TalonFX m_turretMotor;
  private final CANcoder m_encoder;
  private final CustomPositionControlLoop m_customController;
  private final DutyCycleOut m_turretDutyCycleRequest;
  private double currentAngle;
  private double setpoint;

  public Turret() {
    m_turretMotor = new TalonFX(Constants.Turret.kTurretMotorID, "Bus 2");
    m_turretDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
    turretMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turretMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
    turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    turretMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    m_turretMotor.getConfigurator().apply(turretMotorConfig);

    m_encoder = new CANcoder(Constants.Turret.kEncoderID, "Bus 2");
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.kMagnetOffset;
    m_encoder.getConfigurator().apply(encoderConfig);

    double absoluteRotations = getAbsolutePosition();
    double motorRotations = absoluteRotations * Constants.Turret.kGearRatioEncoder;
    m_turretMotor.setPosition(motorRotations);

    m_customController =
        new CustomPositionControlLoop(
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

  public double getAbsolutePosition() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getTurretAngle() {
    double motorRotations = m_turretMotor.getPosition().getValueAsDouble();
    double turretRotations = motorRotations / Constants.Turret.kGearRatioTurretAngleRatio;
    return Units.rotationsToDegrees(turretRotations);
  }

  public void setSetpoint(double degrees) {
    double correctedDegrees =
        MathUtil.clamp(
            degrees, Constants.Turret.kMinAngleDegrees, Constants.Turret.kMaxAngleDegrees);
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
    m_turretMotor.setControl(m_turretDutyCycleRequest);
    m_customController.reset();
  }

  public void resetEncoder() {
    m_encoder.setPosition(0.0);
  }

  public double getDistanceToTarget(Pose2d pose) {
    Translation2d turretPivotField = pose.getTranslation()
        .plus(Constants.Turret.kTurretOffset.rotateBy(pose.getRotation()));
    return Blackbox.getActiveTarget(pose).minus(turretPivotField).getNorm();
  }

  private double getError(double targetAngle, double currentAngle) {
    double error = targetAngle - currentAngle;
    return error;
  }

  private void setMotorOutput(double output) {
    m_turretDutyCycleRequest.Output = output;
    m_turretMotor.setControl(m_turretDutyCycleRequest);
  }

  @Override
  public void periodic() {
    currentAngle = getTurretAngle();
    double smallestError = getError(setpoint, currentAngle);
    double controlLoop = m_customController.calculate(smallestError, currentAngle, setpoint);
    double motorOutput = (controlLoop / 100.0);

    if (currentAngle >= Constants.Turret.kMaxAngleDegrees && motorOutput > 0) {
      motorOutput = 0;
    } else if (currentAngle <= Constants.Turret.kMinAngleDegrees && motorOutput < 0) {
      motorOutput = 0;
    }

    setMotorOutput(motorOutput);

    Logger.recordOutput("Turret/Current Angle", currentAngle);
    Logger.recordOutput("Turret/Absolute", getAbsolutePosition());
    Logger.recordOutput("Turret/Setpoint", setpoint);
    Logger.recordOutput("Turret/At Setpoint", atSetpoint());
    Logger.recordOutput("Turret/Motor Output", motorOutput);
    Logger.recordOutput("Turret/P Value", m_customController.getP());
    Logger.recordOutput("Turret/Position Error", smallestError);
    Logger.recordOutput("Turret/Relative", m_turretMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Turret/StatorCurrent", m_turretMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Turret/SupplyCurrent", m_turretMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Turret/Temperature", m_turretMotor.getDeviceTemp().getValueAsDouble());


  }
}