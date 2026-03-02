package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

/**
 * Subsystem that controls the angular position of the turret.
 *
 * <p>Uses a TalonFX motor on "Bus 1" driven in open-loop duty-cycle mode with output
 * calculated by a {@link CustomPositionControlLoop}. A CANcoder on "Bus 1" provides
 * absolute position feedback; its reading seeds the motor's internal relative encoder at
 * startup. Soft limits prevent the turret from travelling beyond its physical range.
 * Telemetry is published via SmartDashboard.
 */
public class Turret extends SubsystemBase {

  /** TalonFX motor controller that rotates the turret. */
  private final TalonFX m_motor;

  /** CANcoder providing absolute angular position feedback for the turret. */
  private final CANcoder m_encoder;

  /** Custom ramp-based position control loop that produces a percentage motor output. */
  private final CustomPositionControlLoop m_customController;

  /** Phoenix 6 duty-cycle control request sent to the motor each periodic loop. */
  private final DutyCycleOut m_turretDutyCycleRequest;

  /** Current measured turret angle in degrees, updated every periodic loop. */
  private double currentAngle;

  /** Desired turret angle in degrees, clamped to the configured range. */
  private double setpoint;

  /**
   * Constructs and configures the Turret subsystem.
   *
   * <p>Creates the TalonFX motor on "Bus 1" (counter-clockwise positive, brake mode) and the
   * CANcoder on "Bus 1" (counter-clockwise positive, discontinuity point 0.5, magnet offset
   * from constants). Reads the absolute encoder to seed the motor's relative encoder. Initialises
   * the {@link CustomPositionControlLoop} with gains from {@link Constants.Turret}. Both
   * {@code currentAngle} and {@code setpoint} are set to the current turret angle so the
   * mechanism holds still on startup.
   */
  public Turret() {
    m_motor = new TalonFX(Constants.Turret.kTurretMotorID, "Bus 1");
    m_turretDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
    turretMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.getConfigurator().apply(turretMotorConfig);

    m_encoder = new CANcoder(Constants.Turret.kEncoderID, "Bus 1");
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.kMagnetOffset;
    m_encoder.getConfigurator().apply(encoderConfig);

    double absoluteRotations = getAbsolutePosition();
    double motorRotations = absoluteRotations * Constants.Turret.kGearRatioEncoder;
    m_motor.setPosition(motorRotations);

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

  /**
   * Returns the CANcoder's current absolute position in rotations.
   *
   * @return absolute encoder position in rotations
   */
  public double getAbsolutePosition() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Returns the current turret angle derived from the motor's relative encoder.
   *
   * <p>Converts motor rotations to turret degrees using the turret angle gear ratio.
   *
   * @return current turret angle in degrees
   */
  public double getTurretAngle() {
    double motorRotations = m_motor.getPosition().getValueAsDouble();
    double turretRotations = motorRotations / Constants.Turret.kGearRatioTurretAngleRatio;
    return Units.rotationsToDegrees(turretRotations);
  }

  /**
   * Sets the desired turret angle, clamped to the mechanism's safe range.
   *
   * @param degrees target turret angle in degrees; values outside
   *     [{@link Constants.Turret#kMinAngleDegrees}, {@link Constants.Turret#kMaxAngleDegrees}]
   *     are clamped
   */
  public void setSetpoint(double degrees) {
    double correctedDegrees =
        MathUtil.clamp(
            degrees, Constants.Turret.kMinAngleDegrees, Constants.Turret.kMaxAngleDegrees);
    setpoint = correctedDegrees;
  }

  /**
   * Returns the current target turret angle.
   *
   * @return setpoint in degrees
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Returns whether the turret has reached its setpoint within the configured tolerance.
   *
   * @return {@code true} if the position error is within {@link Constants.Turret#kToleranceDegrees}
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
   * Immediately stops the turret motor and resets the control loop state.
   *
   * <p>Sets duty-cycle output to zero and resets internal ramp state in the
   * {@link CustomPositionControlLoop}.
   */
  public void stopMotor() {
    m_turretDutyCycleRequest.Output = 0.0;
    m_motor.setControl(m_turretDutyCycleRequest);
    m_customController.reset();
  }

  /**
   * Resets the CANcoder position to zero.
   *
   * <p>Should be called when the turret is at a known reference position.
   */
  public void resetEncoder() {
    m_encoder.setPosition(0.0);
  }

  /**
   * Computes the angular error between a target and the current turret angle.
   *
   * @param targetAngle  desired angle in degrees
   * @param currentAngle measured angle in degrees
   * @return error in degrees (target - current); positive when the turret is below the target
   */
  private double getError(double targetAngle, double currentAngle) {
    double error = targetAngle - currentAngle;
    return error;
  }

  /**
   * Writes a duty-cycle output directly to the turret motor.
   *
   * @param output motor output fraction in the range [-1.0, 1.0]
   */
  private void setMotorOutput(double output) {
    m_turretDutyCycleRequest.Output = output;
    m_motor.setControl(m_turretDutyCycleRequest);
  }

  /**
   * Periodic loop executed once per scheduler cycle (~20 ms).
   *
   * <p>Reads the current turret angle, runs the custom position control loop to obtain a
   * motor output, applies soft-limit clamping at the angle boundaries, commands the motor,
   * and publishes telemetry to SmartDashboard.
   */
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

    SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
    SmartDashboard.putNumber("Turret/Absolute", getAbsolutePosition());
    SmartDashboard.putNumber("Turret/Setpoint", setpoint);
    SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
    SmartDashboard.putNumber("Turret/Motor Output", motorOutput);
    SmartDashboard.putNumber("Turret/P Value", m_customController.getP());
    SmartDashboard.putNumber("Turret/Position Error", smallestError);
    SmartDashboard.putNumber("Turret/Relative", m_motor.getPosition().getValueAsDouble());
  }
}