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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.Robot;
import team1403.robot.util.CustomPositionControlLoop;

public class Turret extends SubsystemBase {
  private final boolean m_isSim = Robot.isSimulation();

  private TalonFX m_turretMotor;
  private CANcoder m_encoder;
  private CustomPositionControlLoop m_customController;
  private DutyCycleOut m_dutyCycleRequest;

  private double m_currentAngle = 0;
  private double m_setpoint     = 0;

  public Turret() {
    if (!m_isSim) {
      m_turretMotor        = new TalonFX(Constants.Turret.kTurretMotorID, "Bus 2");
      m_dutyCycleRequest   = new DutyCycleOut(0);

      TalonFXConfiguration cfg = new TalonFXConfiguration();
      cfg.MotorOutput.Inverted   = InvertedValue.CounterClockwise_Positive;
      cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      cfg.CurrentLimits.StatorCurrentLimit       = 120;
      cfg.CurrentLimits.StatorCurrentLimitEnable = true;
      cfg.CurrentLimits.SupplyCurrentLimit       = 70;
      cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_turretMotor.getConfigurator().apply(cfg);

      m_encoder = new CANcoder(Constants.Turret.kEncoderID, "Bus 2");
      CANcoderConfiguration encCfg = new CANcoderConfiguration();
      encCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      encCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      encCfg.MagnetSensor.MagnetOffset = Constants.Turret.kMagnetOffset;
      m_encoder.getConfigurator().apply(encCfg);

      double abs = m_encoder.getAbsolutePosition().getValueAsDouble();
      m_turretMotor.setPosition(abs * Constants.Turret.kGearRatioEncoder);

      m_customController = new CustomPositionControlLoop(
          Constants.Turret.kGain, Constants.Turret.kToleranceDegrees,
          Constants.Turret.kRampUpTime, Constants.Turret.kRampDownTime,
          Constants.Turret.kUnitsPerRampTime, Constants.Turret.kMaxSpeed,
          Constants.Turret.kMinSpeed, Constants.Turret.kLoopTime);

      m_currentAngle = getTurretAngle();
      m_setpoint     = m_currentAngle;
    }
  }

  public void setSetpoint(double degrees) {
    m_setpoint = MathUtil.clamp(degrees,
        Constants.Turret.kMinAngleDegrees, Constants.Turret.kMaxAngleDegrees);
  }

  public double getSetpoint()    { return m_setpoint; }

  /** In sim, reports setpoint directly and is always at setpoint. */
  public double getTurretAngle() {
    if (m_isSim) return m_setpoint;
    double rot = m_turretMotor.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(rot / Constants.Turret.kGearRatioTurretAngleRatio);
  }

  public boolean atSetpoint() {
    if (m_isSim) return true;
    return m_customController.isAtPosition();
  }

  public void stopMotor() {
    if (!m_isSim) {
      m_dutyCycleRequest.Output = 0;
      m_turretMotor.setControl(m_dutyCycleRequest);
      m_customController.reset();
    }
    m_setpoint = m_currentAngle;
  }

  @Override
  public void periodic() {
    if (!m_isSim) {
      m_currentAngle = getTurretAngle();
      double error  = m_setpoint - m_currentAngle;
      double output = m_customController.calculate(error, m_currentAngle, m_setpoint) / 100.0;
      if (m_currentAngle >= Constants.Turret.kMaxAngleDegrees && output > 0) output = 0;
      if (m_currentAngle <= Constants.Turret.kMinAngleDegrees && output < 0) output = 0;
      m_dutyCycleRequest.Output = output;
      m_turretMotor.setControl(m_dutyCycleRequest);
    } else {
      m_currentAngle = m_setpoint; // sim: instantly at setpoint
    }
    Logger.recordOutput("Turret/Current Angle", m_currentAngle);
    Logger.recordOutput("Turret/Setpoint",      m_setpoint);
    Logger.recordOutput("Turret/At Setpoint",   atSetpoint());
  }
}
