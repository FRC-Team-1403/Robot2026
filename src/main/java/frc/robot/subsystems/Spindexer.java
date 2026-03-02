package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the spindexer mechanism.
 *
 * <p>Drives a single TalonFX motor in closed-loop velocity mode (VelocityVoltage with FOC)
 * to spin game pieces for indexing or feeding. Handles motor configuration, gear-ratio
 * conversion, PID/feedforward tuning, and AdvantageKit telemetry.
 */
public class Spindexer extends SubsystemBase {

  /** TalonFX motor controller driving the spindexer. */
  private final TalonFX m_spindexerMotor;

  /** Phoenix 6 velocity control request sent to the motor each periodic loop. */
  private final VelocityVoltage m_spindexerVelocityRequest;

  /** Most recently commanded spindexer speed, in RPM at the mechanism (after gear ratio). */
  private double m_spindexerTargetRPM = 0;

  /** Cached status signal for the motor's raw shaft velocity (rotations per second). */
  private final StatusSignal<AngularVelocity> m_spindexerVelocity;

  /**
   * Constructs and configures the Spindexer subsystem.
   *
   * <p>Creates the TalonFX motor and applies a full {@link TalonFXConfiguration} that sets:
   * <ul>
   *   <li>Motor output direction: counter-clockwise positive</li>
   *   <li>Neutral mode: coast</li>
   *   <li>Stator and supply current limits: 40 A each (limits disabled by default)</li>
   *   <li>Slot 0 PID/feedforward gains from {@link Constants.Spindexer}</li>
   * </ul>
   * The velocity status signal is also fetched here for efficient periodic refreshing.
   */
  public Spindexer() {
    m_spindexerMotor = new TalonFX(Constants.Spindexer.m_spindexerID);

    m_spindexerVelocityRequest = new VelocityVoltage(0);
    m_spindexerVelocityRequest.Slot = 0;
    m_spindexerVelocityRequest.EnableFOC = true;

    TalonFXConfiguration spindexerLeaderConfig = new TalonFXConfiguration();
    spindexerLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    spindexerLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerLeaderConfig.CurrentLimits.StatorCurrentLimit = 40;
    spindexerLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    spindexerLeaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
    spindexerLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    Slot0Configs spindexerPIDConfig = new Slot0Configs();
    spindexerPIDConfig.kP = Constants.Spindexer.kP;
    spindexerPIDConfig.kI = Constants.Spindexer.kI;
    spindexerPIDConfig.kD = Constants.Spindexer.kD;
    spindexerPIDConfig.kS = Constants.Spindexer.kS;
    spindexerPIDConfig.kV = Constants.Spindexer.kV;
    spindexerPIDConfig.kA = Constants.Spindexer.kA;
    spindexerLeaderConfig.Slot0 = spindexerPIDConfig;

    m_spindexerMotor.getConfigurator().apply(spindexerLeaderConfig);
    m_spindexerVelocity = m_spindexerMotor.getVelocity();
  }

  /**
   * Commands the spindexer to spin at the given speed using closed-loop velocity control.
   *
   * <p>Converts mechanism RPM to motor-shaft rotations per second via the gear ratio and
   * writes the value to the velocity request object.
   *
   * @param rpm desired mechanism-side speed in RPM; positive spins in the configured positive direction
   */
  public void setSpindexerRPM(double rpm) {
    m_spindexerTargetRPM = rpm;
    m_spindexerVelocityRequest.Velocity = (rpm * Constants.Spindexer.m_spindexerGearRatio) / 60.0;
  }

  /**
   * Stops the spindexer by commanding 0 RPM.
   *
   * <p>Delegates to {@link #setSpindexerRPM(double)} with {@code 0}.
   */
  public void stop() {
    setSpindexerRPM(0);
  }

  /**
   * Returns the current measured speed of the spindexer at the mechanism.
   *
   * <p>Converts the raw motor shaft velocity (rotations per second) to mechanism RPM
   * by applying the inverse of the gear ratio.
   *
   * @return current spindexer speed in RPM
   */
  public double getSpindexerRPM() {
    return (m_spindexerVelocity.getValueAsDouble() * 60.0)
        / Constants.Spindexer.m_spindexerGearRatio;
  }

  /**
   * Returns the most recently commanded target speed.
   *
   * @return target spindexer speed in RPM
   */
  public double getSpindexerTargetRPM() {
    return m_spindexerTargetRPM;
  }

  /**
   * Returns the difference between the target and measured spindexer speeds.
   *
   * @return RPM error (target - actual); positive when the spindexer is slower than desired
   */
  public double getSpindexerRPMError() {
    return m_spindexerTargetRPM - getSpindexerRPM();
  }

  /**
   * Returns whether the spindexer has reached its target speed within tolerance.
   *
   * @return {@code true} if the absolute RPM error is less than {@link Constants.Spindexer#rpmTolerance}
   */
  public boolean isSpindexerAtSpeed() {
    return Math.abs(getSpindexerRPMError()) < Constants.Spindexer.rpmTolerance;
  }

  /**
   * Periodic loop executed once per scheduler cycle (~20 ms).
   *
   * <p>Refreshes the cached velocity status signal, applies the velocity control request
   * to the motor, and publishes comprehensive telemetry to AdvantageKit {@link Logger}.
   */
  @Override
  public void periodic() {
    m_spindexerVelocity.refresh();
    m_spindexerMotor.setControl(m_spindexerVelocityRequest);

    Logger.recordOutput("Spindexer/Target RPM", m_spindexerTargetRPM);
    Logger.recordOutput("Spindexer/Current RPM", getSpindexerRPM());
    Logger.recordOutput("Spindexer/RPM Error", getSpindexerRPMError());
    Logger.recordOutput("Spindexer/At Speed", isSpindexerAtSpeed());
    Logger.recordOutput("Spindexer/Voltage", m_spindexerMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Spindexer/Stator Current", m_spindexerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Spindexer/Supply Current", m_spindexerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Spindexer/Torque Current", m_spindexerMotor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Spindexer/Closed Loop Error", m_spindexerMotor.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput(
        "Spindexer/Closed Loop Output", m_spindexerMotor.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput(
        "Spindexer/Duty Cycle", m_spindexerMotor.getDutyCycle().getValueAsDouble() * 1000);
    Logger.recordOutput(
        "Spindexer/Temperature", m_spindexerMotor.getDeviceTemp().getValueAsDouble());
  }
}