package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the angular position of the shooter hood.
 *
 * <p>Uses a TalonFX motor driven in open-loop duty-cycle mode. The output is the sum of
 * a gravity-compensating {@link ArmFeedforward} term and a {@link CustomPositionControlLoop}
 * output. A CANcoder provides absolute position feedback; its reading is used at startup to
 * seed the motor's internal relative encoder. Soft limits prevent the hood from travelling
 * beyond its physical range.
 */
public class ShooterHood extends SubsystemBase {

  /** TalonFX motor controller that rotates the hood. */
  private final TalonFX m_hoodMotor;

  /** CANcoder providing absolute angular position feedback for the hood. */
  private final CANcoder m_encoder;

  /** Gravity-compensating arm feedforward controller. */
  private ArmFeedforward m_hoodFeedforward;

  /** Phoenix 6 duty-cycle control request sent to the motor each periodic loop. */
  private final DutyCycleOut m_dutyCycleRequest;

  /** Phoenix 6 neutral (coast/brake) request used when stopping the motor. */
  private final NeutralOut m_neutralRequest;

  /** Custom ramp-based position control loop that produces a percentage motor output. */
  private final CustomPositionControlLoop m_customController;

  /** Current measured hood angle in degrees, updated every periodic loop. */
  private double currentAngle;

  /** Desired hood angle in degrees, clamped to the configured range. */
  private double setpoint;

  /**
   * Constructs and configures the ShooterHood subsystem.
   *
   * <p>Creates the TalonFX motor on "Bus 1" (brake mode) and the CANcoder on "Bus 1"
   * (counter-clockwise positive, magnet offset applied). Reads the CANcoder's absolute position
   * to seed the motor's relative encoder so position is retained after a reboot. Initialises
   * the {@link ArmFeedforward} and {@link CustomPositionControlLoop} with gains from
   * {@link Constants.ShooterHood}. Both {@code currentAngle} and {@code setpoint} are set to
   * the current hood angle so the mechanism holds still on startup.
   */
  public ShooterHood() {
    m_hoodMotor = new TalonFX(Constants.ShooterHood.kHoodMotorID, "Bus 1");
    m_encoder = new CANcoder(Constants.ShooterHood.kEncoderID, "Bus 1");
    m_dutyCycleRequest = new DutyCycleOut(0);
    m_neutralRequest = new NeutralOut();
    m_hoodFeedforward =
        new ArmFeedforward(
            Constants.ShooterHood.kS,
            Constants.ShooterHood.kG,
            Constants.ShooterHood.kV,
            Constants.ShooterHood.kA);

    TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_hoodMotor.getConfigurator().apply(hoodMotorConfig);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    config.MagnetSensor.MagnetOffset = -0.89111328125;
    m_encoder.getConfigurator().apply(config);

    double absoluteRotations = getAbsolutePosition();
    double hoodRotations = absoluteRotations * Constants.ShooterHood.kGearRatioEncoder;
    m_hoodMotor.setPosition(hoodRotations);

    m_customController =
        new CustomPositionControlLoop(
            Constants.ShooterHood.kGain,
            Constants.ShooterHood.kToleranceDegrees,
            Constants.ShooterHood.kRampUpTime,
            Constants.ShooterHood.kRampDownTime,
            Constants.ShooterHood.kUnitsPerRampTime,
            Constants.ShooterHood.kMaxSpeed,
            Constants.ShooterHood.kMinSpeed,
            Constants.ShooterHood.kLoopTime);

    currentAngle = getHoodAngle();
    setpoint = currentAngle;
  }

  /**
   * Returns the CANcoder's current absolute position in rotations.
   *
   * @return absolute encoder position in rotations
   */
  public double getAbsolutePosition() {
    return (m_encoder.getAbsolutePosition().getValueAsDouble());
  }

  /**
   * Returns the current hood angle derived from the motor's relative encoder.
   *
   * <p>Converts motor rotations to hood degrees using the hood angle gear ratio.
   *
   * @return current hood angle in degrees
   */
  public double getHoodAngle() {
    double motorRotations = m_hoodMotor.getPosition().getValueAsDouble();
    double hoodRotations = motorRotations / Constants.ShooterHood.kGearRatioHoodAngleRatio;
    return Units.rotationsToDegrees(hoodRotations);
  }

  /**
   * Sets the desired hood angle, clamped to the mechanism's safe range.
   *
   * @param degrees target hood angle in degrees; values outside
   *     [{@link Constants.ShooterHood#kMinAngleDegrees}, {@link Constants.ShooterHood#kMaxAngleDegrees}]
   *     are clamped
   */
  public void setSetpoint(double degrees) {
    double correctedDegrees =
        MathUtil.clamp(
            degrees,
            Constants.ShooterHood.kMinAngleDegrees,
            Constants.ShooterHood.kMaxAngleDegrees);
    setpoint = correctedDegrees;
  }

  /**
   * Returns the current target hood angle.
   *
   * @return setpoint in degrees
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Returns whether the hood has reached its setpoint within the configured tolerance.
   *
   * @return {@code true} if the position error is within {@link Constants.ShooterHood#kToleranceDegrees}
   */
  public boolean atSetpoint() {
    return m_customController.isAtPosition();
  }

  /**
   * Increments the current setpoint by the given offset.
   *
   * <p>The resulting angle is clamped by {@link #setSetpoint(double)}.
   *
   * @param degrees amount to add to the current setpoint, in degrees
   */
  public void adjustSetpoint(double degrees) {
    setSetpoint(setpoint + degrees);
  }

  /**
   * Immediately stops the hood motor using the neutral request and resets the control loop.
   *
   * <p>Applies the {@link NeutralOut} request and resets internal ramp state in the
   * {@link CustomPositionControlLoop}.
   */
  public void stopMotor() {
    m_hoodMotor.setControl(m_neutralRequest);
    m_customController.reset();
  }

  /**
   * Computes the angular error between a target and the current hood angle.
   *
   * @param targetAngle  desired angle in degrees
   * @param currentAngle measured angle in degrees
   * @return error in degrees (target - current); positive when the hood is below the target
   */
  private double getError(double targetAngle, double currentAngle) {
    double error = targetAngle - currentAngle;
    return error;
  }

  /**
   * Writes a duty-cycle output directly to the hood motor.
   *
   * @param output motor output fraction in the range [-1.0, 1.0]
   */
  private void setMotorOutput(double output) {
    m_dutyCycleRequest.Output = output;
    m_hoodMotor.setControl(m_dutyCycleRequest);
  }

  /**
   * Periodic loop executed once per scheduler cycle (~20 ms).
   *
   * <p>Reads the current hood angle, computes the combined feedforward + control-loop motor
   * output, applies soft-limit clamping at the angle boundaries, commands the motor, and
   * publishes telemetry to AdvantageKit {@link Logger}.
   */
  @Override
  public void periodic() {
    currentAngle = getHoodAngle();
    double smallestError = getError(setpoint, currentAngle);
    double controlLoop = m_customController.calculate(smallestError, currentAngle, setpoint);
    double ff = m_hoodFeedforward.calculate(Units.degreesToRadians(currentAngle), 0);
    double motorOutput = ff + controlLoop;

    if (currentAngle >= Constants.ShooterHood.kMaxAngleDegrees && motorOutput > 0) {
      motorOutput = 0;
    } else if (currentAngle <= Constants.ShooterHood.kMinAngleDegrees && motorOutput < 0) {
      motorOutput = 0;
    }

    setMotorOutput(motorOutput / 100.0);

    Logger.recordOutput("Hood/Shooter Hood Current Angle", currentAngle);
    Logger.recordOutput("Hood/Absolute", getAbsolutePosition());
    Logger.recordOutput("Hood/Setpoint", setpoint);
    Logger.recordOutput("Hood/At Setpoint", atSetpoint());
    Logger.recordOutput("Hood/Motor Output", motorOutput);
    Logger.recordOutput("Hood/P Value", m_customController.getP());
    Logger.recordOutput("Hood/Position Error", smallestError);
    Logger.recordOutput("Hood/Relative", m_hoodMotor.getPosition().getValueAsDouble());
  }
}