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
 * Subsystem that controls the indexer mechanism.
 *
 * <p>Drives a single TalonFX motor in closed-loop velocity mode (VelocityVoltage with FOC)
 * to transport game pieces through the robot. Handles motor configuration, gear-ratio
 * conversion, PID/feedforward tuning, and AdvantageKit telemetry.
 */
public class Indexer extends SubsystemBase {

  /** TalonFX motor controller that drives the indexer roller. */
  private final TalonFX m_indexerMotor;

  /** Phoenix 6 velocity control request applied to the motor every periodic loop. */
  private final VelocityVoltage m_indexerVelocityRequest;

  /** Most recently commanded indexer speed, in RPM at the mechanism (after gear ratio). */
  private double m_indexerTargetRPM = 0;

  /** Cached status signal for the motor's raw shaft velocity (rotations per second). */
  private final StatusSignal<AngularVelocity> m_indexerVelocity;

  /**
   * Constructs and configures the Indexer subsystem.
   *
   * <p>Creates the TalonFX motor and applies a full {@link TalonFXConfiguration} that sets:
   * <ul>
   *   <li>Motor output direction: counter-clockwise positive</li>
   *   <li>Neutral mode: coast</li>
   *   <li>Stator and supply current limits: 40 A each (limits disabled by default)</li>
   *   <li>Slot 0 PID/feedforward gains from {@link Constants.Indexer}</li>
   * </ul>
   * The velocity status signal is also fetched here for efficient periodic refreshing.
   */
  public Indexer() {
    m_indexerMotor = new TalonFX(Constants.Indexer.m_indexerID);

    m_indexerVelocityRequest = new VelocityVoltage(0);
    m_indexerVelocityRequest.Slot = 0;
    m_indexerVelocityRequest.EnableFOC = true;

    TalonFXConfiguration indexerLeaderConfig = new TalonFXConfiguration();
    indexerLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerLeaderConfig.CurrentLimits.StatorCurrentLimit = 40;
    indexerLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    indexerLeaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
    indexerLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    Slot0Configs indexerPIDConfig = new Slot0Configs();
    indexerPIDConfig.kP = Constants.Indexer.kP;
    indexerPIDConfig.kI = Constants.Indexer.kI;
    indexerPIDConfig.kD = Constants.Indexer.kD;
    indexerPIDConfig.kS = Constants.Indexer.kS;
    indexerPIDConfig.kV = Constants.Indexer.kV;
    indexerPIDConfig.kA = Constants.Indexer.kA;
    indexerLeaderConfig.Slot0 = indexerPIDConfig;

    m_indexerMotor.getConfigurator().apply(indexerLeaderConfig);
    m_indexerVelocity = m_indexerMotor.getVelocity();
  }

  /**
   * Commands the indexer to spin at the given speed.
   *
   * <p>Stores the target RPM and converts it to motor-shaft rotations per second
   * (accounting for the gear ratio) before writing to the velocity request object.
   *
   * @param rpm desired mechanism-side speed in RPM; positive spins in the configured positive direction
   */
  public void setIndexerRPM(double rpm) {
    m_indexerTargetRPM = rpm;
    m_indexerVelocityRequest.Velocity = (rpm * Constants.Indexer.m_indexerGearRatio) / 60.0;
  }

  /**
   * Stops the indexer by commanding 0 RPM.
   *
   * <p>Delegates to {@link #setIndexerRPM(double)} with {@code 0}.
   */
  public void stop() {
    setIndexerRPM(0);
  }

  /**
   * Returns the current measured speed of the indexer at the mechanism.
   *
   * <p>Converts the raw motor shaft velocity (rotations per second) to mechanism RPM
   * by applying the inverse of the gear ratio.
   *
   * @return current indexer speed in RPM
   */
  public double getIndexerRPM() {
    return (m_indexerVelocity.getValueAsDouble() * 60.0) / Constants.Indexer.m_indexerGearRatio;
  }

  /**
   * Returns the most recently commanded target speed.
   *
   * @return target indexer speed in RPM
   */
  public double getIndexerTargetRPM() {
    return m_indexerTargetRPM;
  }

  /**
   * Returns the difference between the target and measured indexer speeds.
   *
   * @return RPM error (target - actual); positive when the indexer is slower than desired
   */
  public double getIndexerRPMError() {
    return m_indexerTargetRPM - getIndexerRPM();
  }

  /**
   * Returns whether the indexer has reached its target speed within tolerance.
   *
   * @return {@code true} if the absolute RPM error is less than {@link Constants.Indexer#rpmTolerance}
   */
  public boolean isIndexerAtSpeed() {
    return Math.abs(getIndexerRPMError()) < Constants.Indexer.rpmTolerance;
  }

  /**
   * Periodic loop executed once per scheduler cycle (~20 ms).
   *
   * <p>Refreshes the cached velocity status signal, applies the velocity control request
   * to the motor, and publishes comprehensive telemetry to AdvantageKit {@link Logger}.
   */
  @Override
  public void periodic() {
    m_indexerVelocity.refresh();
    m_indexerMotor.setControl(m_indexerVelocityRequest);

    Logger.recordOutput("Indexer/Target RPM", m_indexerTargetRPM);
    Logger.recordOutput("Indexer/Leader RPM", getIndexerRPM());
    Logger.recordOutput("Indexer/RPM Error", getIndexerRPMError());
    Logger.recordOutput("Indexer/At Speed", isIndexerAtSpeed());
    Logger.recordOutput(
        "Indexer/Leader Voltage", m_indexerMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Leader Stator Current", m_indexerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Supply Current", m_indexerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Torque Current", m_indexerMotor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Closed Loop Error", m_indexerMotor.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Closed Loop Output", m_indexerMotor.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Duty Cycle", m_indexerMotor.getDutyCycle().getValueAsDouble() * 1000);
    Logger.recordOutput("Indexer/Leader Temp", m_indexerMotor.getDeviceTemp().getValueAsDouble());
  }
}