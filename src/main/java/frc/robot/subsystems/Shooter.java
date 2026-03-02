package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the shooter flywheel mechanism.
 *
 * <p>Uses three TalonFX motors in a leader/follower configuration. The leader runs in
 * closed-loop velocity mode (VelocityVoltage with FOC) or open-loop duty-cycle mode;
 * the two followers are set to oppose the leader's direction. Handles motor configuration,
 * gear-ratio conversion, PID/feedforward tuning, and AdvantageKit telemetry.
 */
public class Shooter extends SubsystemBase {

  /** TalonFX leader motor that receives the active control request. */
  private final TalonFX m_flywheelLeader;

  /** First TalonFX follower motor, set to oppose the leader's output. */
  private final TalonFX m_flywheelFollower;

  /** Second TalonFX follower motor, set to oppose the leader's output. */
  private final TalonFX m_flywheelFollower2;

  /** Phoenix 6 velocity control request used when in closed-loop mode. */
  private final VelocityVoltage m_flywheelVelocityRequest;

  /** Phoenix 6 duty-cycle control request used when in open-loop mode. */
  private final DutyCycleOut m_flywheelDutyCycleRequest;

  /** Most recently commanded flywheel speed, in RPM at the mechanism (after gear ratio). */
  private double m_flywheelTargetRPM = 0;

  /** Most recently commanded open-loop duty cycle (range -1.0 to 1.0). */
  private double m_flywheelTargetDutyCycle = 0;

  /** {@code true} when the leader should use closed-loop velocity control; {@code false} for duty-cycle. */
  private boolean m_flywheelUseVelocityControl = true;

  /** Cached velocity status signal for the leader motor (rotations per second). */
  @SuppressWarnings("all")
  private final StatusSignal m_flywheelLeaderVelocity;

  /** Cached velocity status signal for the first follower motor (rotations per second). */
  @SuppressWarnings("all")
  private final StatusSignal m_flywheelFollowerVelocity;

  /** Cached velocity status signal for the second follower motor (rotations per second). */
  @SuppressWarnings("all")
  private final StatusSignal m_flywheelFollower2Velocity;

  /**
   * Constructs and configures the Shooter subsystem.
   *
   * <p>Creates three TalonFX motors and applies individual {@link TalonFXConfiguration}
   * objects. The leader is configured counter-clockwise positive with Slot 0 PID/feedforward
   * gains from {@link Constants.Shooter} and 40 A current limits enabled. Both followers
   * are configured clockwise positive with matching current limits and set to oppose the
   * leader via the {@link Follower} control request. Velocity status signals are fetched
   * from all three motors for efficient periodic refreshing.
   */
  public Shooter() {
    m_flywheelLeader = new TalonFX(Constants.Shooter.flywheelLeaderID);
    m_flywheelFollower = new TalonFX(Constants.Shooter.flywheelFollowerID);
    m_flywheelFollower2 = new TalonFX(Constants.Shooter.flywheelFollower2ID);
    m_flywheelVelocityRequest = new VelocityVoltage(0);
    m_flywheelVelocityRequest.Slot = 0;
    m_flywheelVelocityRequest.EnableFOC = true;
    m_flywheelDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
    flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit = 40;
    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    Slot0Configs flywheelPIDConfig = new Slot0Configs();
    flywheelPIDConfig.kP = Constants.Shooter.kP;
    flywheelPIDConfig.kI = Constants.Shooter.kI;
    flywheelPIDConfig.kD = Constants.Shooter.kD;
    flywheelPIDConfig.kS = Constants.Shooter.kS;
    flywheelPIDConfig.kV = Constants.Shooter.kV;
    flywheelPIDConfig.kA = Constants.Shooter.kA;
    flywheelLeaderConfig.Slot0 = flywheelPIDConfig;
    m_flywheelLeader.getConfigurator().apply(flywheelLeaderConfig);

    TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
    flywheelFollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 40;
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_flywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
    m_flywheelFollower.setControl(
        new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));

    TalonFXConfiguration flywheelFollower2Config = new TalonFXConfiguration();
    flywheelFollower2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelFollower2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelFollower2Config.CurrentLimits.StatorCurrentLimit = 40;
    flywheelFollower2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelFollower2Config.CurrentLimits.SupplyCurrentLimit = 40;
    flywheelFollower2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_flywheelFollower2.getConfigurator().apply(flywheelFollower2Config);
    m_flywheelFollower2.setControl(
        new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));

    m_flywheelLeaderVelocity = m_flywheelLeader.getVelocity();
    m_flywheelFollowerVelocity = m_flywheelFollower.getVelocity();
    m_flywheelFollower2Velocity = m_flywheelFollower2.getVelocity();
  }

  /**
   * Commands the flywheel to spin at the given speed using closed-loop velocity control.
   *
   * <p>Converts mechanism RPM to motor-shaft rotations per second via the gear ratio,
   * writes the value to the velocity request, and switches the control mode to velocity control.
   *
   * @param rpm desired mechanism-side flywheel speed in RPM
   */
  public void setFlywheelTargetRPM(double rpm) {
    m_flywheelTargetRPM = rpm;
    m_flywheelVelocityRequest.Velocity = (rpm * Constants.Shooter.flywheelGearRatio) / 60.0;
    m_flywheelUseVelocityControl = true;
  }

  /**
   * Commands the flywheel motor using open-loop duty-cycle control.
   *
   * <p>Stores the duty cycle, writes it to the duty-cycle request, and switches the
   * control mode to open-loop.
   *
   * @param dutyCycle motor output fraction in the range [-1.0, 1.0]
   */
  public void setFlywheelTargetPower(double dutyCycle) {
    m_flywheelTargetDutyCycle = dutyCycle;
    m_flywheelDutyCycleRequest.Output = dutyCycle;
    m_flywheelUseVelocityControl = false;
  }

  /**
   * Stops the flywheel by commanding 0 RPM in velocity-control mode.
   *
   * <p>Delegates to {@link #setFlywheelTargetRPM(double)} with {@code 0}.
   */
  public void stop() {
    setFlywheelTargetRPM(0);
  }

  /**
   * Returns the current measured speed of the leader flywheel motor at the mechanism.
   *
   * @return leader motor speed in RPM
   */
  public double getFlywheelLeaderRPM() {
    return (m_flywheelLeaderVelocity.getValueAsDouble() * 60.0)
        / Constants.Shooter.flywheelGearRatio;
  }

  /**
   * Returns the current measured speed of the first follower flywheel motor at the mechanism.
   *
   * @return first follower motor speed in RPM
   */
  public double getFlywheelFollowerRPM() {
    return (m_flywheelFollowerVelocity.getValueAsDouble() * 60.0)
        / Constants.Shooter.flywheelGearRatio;
  }

  /**
   * Returns the current measured speed of the second follower flywheel motor at the mechanism.
   *
   * @return second follower motor speed in RPM
   */
  public double getFlywheelFollower2RPM() {
    return (m_flywheelFollower2Velocity.getValueAsDouble() * 60.0)
        / Constants.Shooter.flywheelGearRatio;
  }

  /**
   * Returns the most recently commanded target flywheel speed.
   *
   * @return target flywheel speed in RPM
   */
  public double getFlywheelTargetRPM() {
    return m_flywheelTargetRPM;
  }

  /**
   * Returns the difference between the target and the leader's measured flywheel speed.
   *
   * @return RPM error (target - leader actual); positive when the flywheel is slower than desired
   */
  public double getFlywheelRPMError() {
    return m_flywheelTargetRPM - getFlywheelLeaderRPM();
  }

  /**
   * Returns whether the flywheel has reached its target speed within tolerance.
   *
   * @return {@code true} if the absolute RPM error is less than {@link Constants.Shooter#rpmTolerance}
   */
  public boolean isFlywheelAtSpeed() {
    return Math.abs(getFlywheelRPMError()) < Constants.Shooter.rpmTolerance;
  }

  /**
   * Returns the most recently commanded open-loop duty cycle.
   *
   * @return target duty cycle in the range [-1.0, 1.0]
   */
  public double getFlywheelTargetDutyCycle() {
    return m_flywheelTargetDutyCycle;
  }

  /**
   * Periodic loop executed once per scheduler cycle (~20 ms).
   *
   * <p>Refreshes all three cached velocity status signals, applies either the velocity or
   * duty-cycle control request to the leader motor, and publishes comprehensive telemetry
   * to AdvantageKit {@link Logger}.
   */
  @Override
  public void periodic() {
    m_flywheelLeaderVelocity.refresh();
    m_flywheelFollowerVelocity.refresh();
    m_flywheelFollower2Velocity.refresh();

    if (m_flywheelUseVelocityControl) {
      m_flywheelLeader.setControl(m_flywheelVelocityRequest);
    } else {
      m_flywheelLeader.setControl(m_flywheelDutyCycleRequest);
    }

    Logger.recordOutput("Flywheel/Target RPM", m_flywheelTargetRPM);
    Logger.recordOutput("Flywheel/Leader RPM", getFlywheelLeaderRPM());
    Logger.recordOutput("Flywheel/Follower RPM", getFlywheelFollowerRPM());
    Logger.recordOutput("Flywheel/Follower2 RPM", getFlywheelFollower2RPM());
    Logger.recordOutput("Flywheel/RPM Error", getFlywheelRPMError());
    Logger.recordOutput("Flywheel/At Speed", isFlywheelAtSpeed());
    Logger.recordOutput("Flywheel/Target Duty Cycle", m_flywheelTargetDutyCycle);
    Logger.recordOutput(
        "Flywheel/Leader Voltage", m_flywheelLeader.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Leader Stator Current", m_flywheelLeader.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Follower Stator Current",
        m_flywheelFollower.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Follower2 Stator Current",
        m_flywheelFollower2.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Supply Current", m_flywheelLeader.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Torque Current", m_flywheelLeader.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Closed Loop Error", m_flywheelLeader.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Closed Loop Output", m_flywheelLeader.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Duty Cycle", m_flywheelLeader.getDutyCycle().getValueAsDouble() * 1000);
    Logger.recordOutput(
        "Flywheel/Leader Temp", m_flywheelLeader.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Follower Temp", m_flywheelFollower.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel/Follower2 Temp", m_flywheelFollower2.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Flywheel/Using Velocity Control", m_flywheelUseVelocityControl);
  }
}