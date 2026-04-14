package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class ShooterHood extends SubsystemBase {
  private final TalonFX m_hoodMotor;
  private final PositionVoltage m_positionVoltageRequest;
  private final NeutralOut m_neutralRequest;
  private double currentAngle;
  private double setpoint;

  public ShooterHood() {
    m_hoodMotor = new TalonFX(Constants.ShooterHood.kHoodMotorID, "Bus 2");
    m_positionVoltageRequest = new PositionVoltage(0);
    m_neutralRequest = new NeutralOut();

    TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
    hoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    Slot0Configs hoodPIDConfigs = new Slot0Configs();
    hoodPIDConfigs.kP = Constants.ShooterHood.kP;
    hoodPIDConfigs.kI = Constants.ShooterHood.kI;
    hoodPIDConfigs.kD = Constants.ShooterHood.kD;
    hoodPIDConfigs.kS = Constants.ShooterHood.kS;
    hoodPIDConfigs.kV = Constants.ShooterHood.kV;
    hoodPIDConfigs.kA = Constants.ShooterHood.kA;
    hoodPIDConfigs.kG = Constants.ShooterHood.kG;
    hoodPIDConfigs.GravityType = GravityTypeValue.Elevator_Static;
    hoodPIDConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    m_hoodMotor.getConfigurator().apply(hoodMotorConfig);
    m_hoodMotor.getConfigurator().apply(hoodPIDConfigs);

    m_hoodMotor.setPosition(0);

    currentAngle = getHoodAngle();
    setpoint = currentAngle;
  }

  public double getHoodAngle() {
    double motorRotations = m_hoodMotor.getPosition().getValueAsDouble();
    double hoodRotations = motorRotations / Constants.ShooterHood.kGearRatioHoodAngleRatio;
    return Units.rotationsToDegrees(hoodRotations);
  }

  public void setSetpoint(double degrees) {
    double correctedDegrees =
        MathUtil.clamp(
            degrees,
            Constants.ShooterHood.kMinAngleDegrees,
            Constants.ShooterHood.kMaxAngleDegrees);
    setpoint = correctedDegrees;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public boolean atSetpoint() {
    return Math.abs(getSetpoint() - getHoodAngle()) < 2.0;
  }

  public void adjustSetpoint(double degrees) {
    setSetpoint(setpoint + degrees);
  }

  public void stopMotor() {
    m_hoodMotor.setControl(m_neutralRequest);
  }

  @Override
  public void periodic() {
    currentAngle = getHoodAngle();

    double setpointRotations = Units.degreesToRotations(setpoint) * Constants.ShooterHood.kGearRatioHoodAngleRatio;
    m_hoodMotor.setControl(m_positionVoltageRequest.withPosition(setpointRotations));

    Logger.recordOutput("Hood/Shooter Hood Current Angle", currentAngle);
    // Logger.recordOutput("Hood/Absolute", getAbsolutePosition());
    // Logger.recordOutput("Hood/Setpoint", setpoint);
    // Logger.recordOutput("Hood/At Setpoint", atSetpoint());
    // Logger.recordOutput("Hood/Relative", m_hoodMotor.getPosition().getValueAsDouble());
    // Logger.recordOutput("Hood/StatorCurrent", m_hoodMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Hood/SupplyCurrent", m_hoodMotor.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput("Hood/Device Temperature Temperature", m_hoodMotor.getDeviceTemp().getValueAsDouble());
  }
}

// package team1403.robot.subsystems;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import team1403.robot.Constants;

// public class ShooterHood extends SubsystemBase {
//   private final TalonFX m_hoodMotor;
//   private final CANcoder m_encoder;
//   private final PositionVoltage m_positionVoltageRequest;
//   private final NeutralOut m_neutralRequest;
//   private double currentAngle;
//   private double setpoint;

//   public ShooterHood() {
//     m_hoodMotor = new TalonFX(Constants.ShooterHood.kHoodMotorID, "Bus 2");
//     m_encoder = new CANcoder(Constants.ShooterHood.kEncoderID, "Bus 2");
//     m_positionVoltageRequest = new PositionVoltage(0);
//     m_neutralRequest = new NeutralOut();

//     TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
//     hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//     hoodMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
//     hoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
//     hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
//     hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//     hoodMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
//     hoodMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

//     Slot0Configs hoodPIDConfigs = new Slot0Configs();
//     hoodPIDConfigs.kP = Constants.ShooterHood.kP;
//     hoodPIDConfigs.kI = Constants.ShooterHood.kI;
//     hoodPIDConfigs.kD = Constants.ShooterHood.kD;
//     hoodPIDConfigs.kS = Constants.ShooterHood.kS;
//     hoodPIDConfigs.kV = Constants.ShooterHood.kV;
//     hoodPIDConfigs.kA = Constants.ShooterHood.kA;
//     hoodPIDConfigs.kG = Constants.ShooterHood.kG;
//     hoodPIDConfigs.GravityType = GravityTypeValue.Elevator_Static;
//     hoodPIDConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

//     m_hoodMotor.getConfigurator().apply(hoodMotorConfig);
//     m_hoodMotor.getConfigurator().apply(hoodPIDConfigs);

//     CANcoderConfiguration config = new CANcoderConfiguration();
//     config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
//     config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
//     config.MagnetSensor.MagnetOffset = Constants.ShooterHood.kMagnetOffset;

//     m_encoder.getConfigurator().apply(config);

//     double absoluteRotations = getAbsolutePosition();
//     double hoodRotations = absoluteRotations * Constants.ShooterHood.kGearRatioEncoder;
//     m_hoodMotor.setPosition(hoodRotations);

//     currentAngle = getHoodAngle();
//     setpoint = currentAngle;
//   }

//   public double getAbsolutePosition() {
//     return (m_encoder.getAbsolutePosition().getValueAsDouble());
//   }

//   public double getHoodAngle() {
//     double motorRotations = m_hoodMotor.getPosition().getValueAsDouble();
//     double hoodRotations = motorRotations / Constants.ShooterHood.kGearRatioHoodAngleRatio;
//     return Units.rotationsToDegrees(hoodRotations);
//   }

//   public void setSetpoint(double degrees) {
//     double correctedDegrees =
//         MathUtil.clamp(
//             degrees,
//             Constants.ShooterHood.kMinAngleDegrees,
//             Constants.ShooterHood.kMaxAngleDegrees);
//     setpoint = correctedDegrees;
//   }

//   public double getSetpoint() {
//     return setpoint;
//   }

//   public boolean atSetpoint() {
//     return Math.abs(getSetpoint() - getHoodAngle()) < 2.0;
//   }

//   public void adjustSetpoint(double degrees) {
//     setSetpoint(setpoint + degrees);
//   }

//   public void stopMotor() {
//     m_hoodMotor.setControl(m_neutralRequest);
//   }

//   @Override
//   public void periodic() {
//     currentAngle = getHoodAngle();

//     double setpointRotations = Units.degreesToRotations(setpoint) * Constants.ShooterHood.kGearRatioHoodAngleRatio;
//     m_hoodMotor.setControl(m_positionVoltageRequest.withPosition(setpointRotations));

//     Logger.recordOutput("Hood/Shooter Hood Current Angle", currentAngle);
//     // Logger.recordOutput("Hood/Absolute", getAbsolutePosition());
//     // Logger.recordOutput("Hood/Setpoint", setpoint);
//     // Logger.recordOutput("Hood/At Setpoint", atSetpoint());
//     // Logger.recordOutput("Hood/Relative", m_hoodMotor.getPosition().getValueAsDouble());
//     // Logger.recordOutput("Hood/StatorCurrent", m_hoodMotor.getStatorCurrent().getValueAsDouble());
//     // Logger.recordOutput("Hood/SupplyCurrent", m_hoodMotor.getSupplyCurrent().getValueAsDouble());
//     // Logger.recordOutput("Hood/Device Temperature Temperature", m_hoodMotor.getDeviceTemp().getValueAsDouble());
//   }
// }