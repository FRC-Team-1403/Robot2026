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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the angular position of the intake wrist joint.
 *
 * <p>Uses a TalonFX motor driven in open-loop duty-cycle mode with output calculated by a
 * {@link CustomPositionControlLoop}. A CANcoder provides absolute position feedback that is
 * converted to degrees via the wrist gear ratio. Soft limits prevent the mechanism from
 * travelling beyond its physical range of motion.
 */
public class IntakeWrist extends SubsystemBase {

  /** TalonFX motor controller that rotates the wrist joint. */
  private final TalonFX m_motor;

  /** CANcoder providing absolute angular position feedback for the wrist. */
  private final CANcoder m_encoder;

  /** Custom ramp-based position control loop that produces a percentage motor output. */
  private final CustomPositionControlLoop m_customController;

  /** Phoenix 6 duty-cycle control request sent to the motor each periodic loop. */
  private final DutyCycleOut m_wristDutyCycleRequest;

  /** Current measured wrist angle in degrees, updated every periodic loop. */
  private double currentAngle;

  /** Desired wrist angle in degrees, clamped to the configured range. */
  private double setpoint;

  /**
   * Constructs and configures the IntakeWrist subsystem.
   *
   * <p>Applies TalonFX configuration (counter-clockwise positive, brake mode) and CANcoder
   * configuration (counter-clockwise positive sensor direction). Resets the encoder to zero,
   * then initialises the {@link CustomPositionControlLoop} with gains from
   * {@link Constants.IntakeWrist}. Both {@code currentAngle} and {@code setpoint} are
   * initialised to the current wrist position so the mechanism holds still on startup.
   */
  public IntakeWrist() {
    m_motor = new TalonFX(Constants.IntakeWrist.kWristMotorID);

    m_wristDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor.getConfigurator().apply(wristMotorConfig);

    m_encoder = new CANcoder(Constants.IntakeWrist.kEncoderID);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    m_encoder.getConfigurator().apply(encoderConfig);

    resetEncoder();

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

  /**
   * Returns the current wrist angle derived from the CANcoder position.
   *
   * <p>Converts encoder rotations to mechanism degrees using the wrist gear ratio.
   *
   * @return current wrist angle in degrees
   */
  public double getWristAngle() {
    double rotations = m_encoder.getPosition().getValueAsDouble();
    double degrees = rotations / Constants.IntakeWrist.kGearRatio * 360.0;
    return degrees;
  }

  /**
   * Sets the desired wrist angle, clamped to the mechanism's safe range.
   *
   * @param degrees target wrist angle in degrees; values outside
   *     [{@link Constants.IntakeWrist#kMinAngleDegrees}, {@link Constants.IntakeWrist#kMaxAngleDegrees}]
   *     are clamped
   */
  public void setSetpoint(double degrees) {
    double correctedDegrees =
        MathUtil.clamp(
            degrees,
            Constants.IntakeWrist.kMinAngleDegrees,
            Constants.IntakeWrist.kMaxAngleDegrees);
    setpoint = correctedDegrees;
  }

  /**
   * Returns the current target wrist angle.
   *
   * @return setpoint in degrees
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Returns whether the wrist has reached its setpoint within the configured tolerance.
   *
   * @return {@code true} if the position error is within {@link Constants.IntakeWrist#kToleranceDegrees}
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
   * Immediately stops the wrist motor and resets the control loop state.
   *
   * <p>Sets duty-cycle output to zero and resets internal ramp state in the
   * {@link CustomPositionControlLoop}.
   */
  public void stopMotor() {
    m_wristDutyCycleRequest.Output = 0.0;
    m_motor.setControl(m_wristDutyCycleRequest);
    m_customController.reset();
  }

  /**
   * Resets the CANcoder position to zero.
   *
   * <p>Should be called when the wrist is at a known reference position.
   */
  public void resetEncoder() {
    m_encoder.setPosition(0.0);
  }

  /**
   * Computes the angular error between a target and the current wrist angle.
   *
   * @param targetAngle  desired angle in degrees
   * @param currentAngle measured angle in degrees
   * @return error in degrees (target - current); positive when the wrist is below the target
   */
  private double getError(double targetAngle, double currentAngle) {
    double error = targetAngle - currentAngle;
    return error;
  }

  /**
   * Writes a duty-cycle output directly to the wrist motor.
   *
   * @param output motor output fraction in the range [-1.0, 1.0]
   */
  private void setMotorOutput(double output) {
    m_wristDutyCycleRequest.Output = output;
    m_motor.setControl(m_wristDutyCycleRequest);
  }

  /**
   * Periodic loop executed once per scheduler cycle (~20 ms).
   *
   * <p>Reads the current wrist angle, runs the custom position control loop to obtain a
   * motor output, applies soft-limit clamping at the angle boundaries, commands the motor,
   * and publishes telemetry to AdvantageKit {@link Logger}.
   */
  @Override
  public void periodic() {
    currentAngle = getWristAngle();

    double smallestError = getError(setpoint, currentAngle);
    double motorOutput = m_customController.calculate(smallestError, currentAngle, setpoint);

    setMotorOutput(motorOutput / 100.0);

    if (currentAngle >= Constants.IntakeWrist.kMaxAngleDegrees && motorOutput > 0) {
      motorOutput = 0;
    } else if (currentAngle <= Constants.IntakeWrist.kMinAngleDegrees && motorOutput < 0) {
      motorOutput = 0;
    }

    Logger.recordOutput("IntakeWrist/Intake Wrist Current Angle", currentAngle);
    Logger.recordOutput("IntakeWrist/Setpoint", setpoint);
    Logger.recordOutput("IntakeWrist/At Setpoint", atSetpoint());
    Logger.recordOutput("IntakeWrist/Motor Output", motorOutput);
    Logger.recordOutput("IntakeWrist/P Value", m_customController.getP());
    Logger.recordOutput("IntakeWrist/Position Error", smallestError);
    Logger.recordOutput(
        "IntakeWrist/Encoder Rotations", m_encoder.getPosition().getValueAsDouble());
  }
}