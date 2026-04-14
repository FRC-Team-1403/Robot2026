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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.util.Blackbox;

public class Turret extends SubsystemBase {
  private final TalonFX m_turretMotor;
  private final CANcoder m_encoder;
  private final PositionVoltage m_positionVoltageRequest;
  private final NeutralOut m_neutralRequest;
  private final DutyCycleOut m_turretDutyCycleRequest;
  private double currentAngle;
  private double setpoint;

  public Turret() {
    m_turretMotor = new TalonFX(Constants.Turret.kTurretMotorID, "Bus 2");
    m_positionVoltageRequest = new PositionVoltage(0);
    m_neutralRequest = new NeutralOut();
    m_turretDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
    turretMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turretMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
    turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    turretMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    Slot0Configs turretPIDConfigs = new Slot0Configs();
    turretPIDConfigs.kP = Constants.Turret.kP;
    turretPIDConfigs.kI = Constants.Turret.kI;
    turretPIDConfigs.kD = Constants.Turret.kD;
    turretPIDConfigs.kS = Constants.Turret.kS;
    turretPIDConfigs.kV = Constants.Turret.kV;
    turretPIDConfigs.kA = Constants.Turret.kA;
    turretPIDConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    m_turretMotor.getConfigurator().apply(turretMotorConfig);
    m_turretMotor.getConfigurator().apply(turretPIDConfigs);

    m_encoder = new CANcoder(Constants.Turret.kEncoderID, "Bus 2");
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.kMagnetOffset;
    m_encoder.getConfigurator().apply(encoderConfig);

    double absoluteRotations = getAbsolutePosition();
    double motorRotations = absoluteRotations * Constants.Turret.kGearRatioEncoder;
    m_turretMotor.setPosition(motorRotations);

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
    return Math.abs(setpoint - currentAngle) <= Constants.Turret.kToleranceDegrees;
  }

  public void adjustSetpoint(double degrees) {
    setSetpoint(setpoint + degrees);
  }

  public void stopMotor() {
    m_turretMotor.setControl(m_neutralRequest);
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

  public void setMotorOutput(double output) {
    m_turretDutyCycleRequest.Output = output;
    m_turretMotor.setControl(m_turretDutyCycleRequest);
  }

  private double getSpringFeedforward(double angleDegrees) {
    double displacement = angleDegrees - Constants.Turret.kSpringNeutralAngle;
    double sign = Math.signum(displacement);
    return Constants.Turret.kSpringK * sign * Constants.Turret.kSpringForce;
  }

  @Override
  public void periodic() {
    currentAngle = getTurretAngle();

    double setpointRotations = Units.degreesToRotations(setpoint) * Constants.Turret.kGearRatioTurretAngleRatio;

    double smallestError = getError(setpoint, currentAngle);

    double ff = getSpringFeedforward(currentAngle);

    if (currentAngle >= Constants.Turret.kMaxAngleDegrees && smallestError > 0) {
      m_turretMotor.setControl(m_neutralRequest);
    } else if (currentAngle <= Constants.Turret.kMinAngleDegrees && smallestError < 0) {
      m_turretMotor.setControl(m_neutralRequest);
    } else {
      m_turretMotor.setControl(m_positionVoltageRequest.withPosition(setpointRotations).withFeedForward(ff));
    }

    Logger.recordOutput("Turret/Turret Current Angle", currentAngle);
    // Logger.recordOutput("Turret/Absolute", getAbsolutePosition());
    // Logger.recordOutput("Turret/Setpoint", setpoint);
    // Logger.recordOutput("Turret/At Setpoint", atSetpoint());
    // Logger.recordOutput("Turret/Position Error", smallestError);
    // Logger.recordOutput("Turret/Relative", m_turretMotor.getPosition().getValueAsDouble());
    // Logger.recordOutput("Turret/StatorCurrent", m_turretMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Turret/SupplyCurrent", m_turretMotor.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput("Turret/Temperature", m_turretMotor.getDeviceTemp().getValueAsDouble());
  }
}