package team1403.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.Robot;
import team1403.robot.util.CustomPositionControlLoop;

public class ShooterHood extends SubsystemBase {
  private final boolean m_isSim = Robot.isSimulation();

  private TalonFX m_hoodMotor;
  private CANcoder m_encoder;
  private ArmFeedforward m_hoodFeedforward;
  private DutyCycleOut m_dutyCycleRequest;
  private NeutralOut m_neutralRequest;
  private CustomPositionControlLoop m_customController;

  private double m_currentAngle = 0;
  private double m_setpoint     = 0;

  public ShooterHood() {
    if (!m_isSim) {
      m_hoodMotor = new TalonFX(Constants.ShooterHood.kHoodMotorID, "Bus 2");
      m_encoder   = new CANcoder(Constants.ShooterHood.kEncoderID, "Bus 2");
      m_dutyCycleRequest = new DutyCycleOut(0);
      m_neutralRequest   = new NeutralOut();
      m_hoodFeedforward  = new ArmFeedforward(
          Constants.ShooterHood.kS, Constants.ShooterHood.kG,
          Constants.ShooterHood.kV, Constants.ShooterHood.kA);

      TalonFXConfiguration cfg = new TalonFXConfiguration();
      cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
      cfg.CurrentLimits.StatorCurrentLimit       = 120;
      cfg.CurrentLimits.StatorCurrentLimitEnable = true;
      cfg.CurrentLimits.SupplyCurrentLimit       = 70;
      cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_hoodMotor.getConfigurator().apply(cfg);

      CANcoderConfiguration encCfg = new CANcoderConfiguration();
      encCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      encCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
      encCfg.MagnetSensor.MagnetOffset = Constants.ShooterHood.kMagnetOffset;
      m_encoder.getConfigurator().apply(encCfg);

      double abs = m_encoder.getAbsolutePosition().getValueAsDouble();
      m_hoodMotor.setPosition(abs * Constants.ShooterHood.kGearRatioEncoder);

      m_customController = new CustomPositionControlLoop(
          Constants.ShooterHood.kGain, Constants.ShooterHood.kToleranceDegrees,
          Constants.ShooterHood.kRampUpTime, Constants.ShooterHood.kRampDownTime,
          Constants.ShooterHood.kUnitsPerRampTime, Constants.ShooterHood.kMaxSpeed,
          Constants.ShooterHood.kMinSpeed, Constants.ShooterHood.kLoopTime);

      m_currentAngle = getHoodAngle();
      m_setpoint     = m_currentAngle;
    }
  }

  public void setSetpoint(double degrees) {
    m_setpoint = MathUtil.clamp(degrees,
        Constants.ShooterHood.kMinAngleDegrees, Constants.ShooterHood.kMaxAngleDegrees);
  }

  public double getSetpoint()    { return m_setpoint; }

  /** In sim, reports setpoint directly and is always at setpoint. */
  public double getHoodAngle()   {
    if (m_isSim) return m_setpoint;
    double rot = m_hoodMotor.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(rot / Constants.ShooterHood.kGearRatioHoodAngleRatio);
  }

  public boolean atSetpoint() {
    if (m_isSim) return true;
    return m_customController.isAtPosition();
  }

  public void stopMotor() {
    if (!m_isSim) { m_hoodMotor.setControl(m_neutralRequest); m_customController.reset(); }
  }

  @Override
  public void periodic() {
    if (!m_isSim) {
      m_currentAngle = getHoodAngle();
      double error  = m_setpoint - m_currentAngle;
      double output = m_customController.calculate(error, m_currentAngle, m_setpoint);
      double ff     = m_hoodFeedforward.calculate(Units.degreesToRadians(m_currentAngle), 0);
      double motor  = (ff + output) / 100.0;
      if (m_currentAngle >= Constants.ShooterHood.kMaxAngleDegrees && motor > 0) motor = 0;
      if (m_currentAngle <= Constants.ShooterHood.kMinAngleDegrees && motor < 0) motor = 0;
      m_dutyCycleRequest.Output = motor;
      m_hoodMotor.setControl(m_dutyCycleRequest);
    } else {
      m_currentAngle = m_setpoint; // sim: instantly at setpoint
    }
    Logger.recordOutput("Hood/Current Angle", m_currentAngle);
    Logger.recordOutput("Hood/Setpoint",      m_setpoint);
    Logger.recordOutput("Hood/At Setpoint",   atSetpoint());
  }
}
