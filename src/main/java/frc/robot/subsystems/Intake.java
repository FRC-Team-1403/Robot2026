package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the robot's intake mechanism.
 *
 * <p>Drives a single TalonFX motor that supports two control modes:
 * closed-loop velocity control (VelocityVoltage with FOC) and open-loop
 * duty-cycle control. Handles motor configuration, gear-ratio conversion,
 * PID/feedforward tuning, and AdvantageKit telemetry.
 */
public class Intake extends SubsystemBase {

  /** TalonFX motor controller driving the intake roller. */
  private final TalonFX m_intake;

  /** Phoenix 6 velocity control request used when in closed-loop mode. */
  private final VelocityVoltage m_intakeVelocityRequest;

  /** Phoenix 6 duty-cycle control request used when in open-loop mode. */
  private final DutyCycleOut m_intakelDutyCycleRequest;

  /** Most recently commanded intake speed, in RPM at the mechanism (after gear ratio). */
  private double m_intakeTargetRPM = 0;

  /** Most recently commanded open-loop duty cycle (range -1.0 to 1.0). */
  private double m_intakeTargetDutyCycle = 0;

  /** {@code true} when the motor should be driven with closed-loop velocity control; {@code false} for duty-cycle. */
  private boolean m_intakeUseVelocityControl = true;

  /** Cached status signal for the motor's raw shaft velocity (rotations per second). */
  private final StatusSignal<AngularVelocity> m_intakeVelocity;

  /**
   * Constructs and configures the Intake subsystem.
   *
   * <p>Creates the TalonFX motor and applies a full {@link TalonFXConfiguration} that sets:
   * <ul>
   *   <li>Motor output direction: counter-clockwise positive</li>
   *   <li>Neutral mode: coast</li>
   *   <li>Stator and supply current limits: 40 A each (limits disabled by default)</li>
   *   <li>Slot 0 PID/feedforward gains from {@link Constants.Intake}</li>
   * </ul>
   * Both control request objects are initialised, and the velocity status signal is
   * fetched for efficient periodic refreshing.
   */
  public Intake() {
    m_intake = new TalonFX(Constants.Intake.m_intakeID);

    m_intakeVelocityRequest = new VelocityVoltage(0);
    m_intakeVelocityRequest.Slot = 0;
    m_intakeVelocityRequest.EnableFOC = true;
    m_intakelDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 40;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    Slot0Configs intakePIDConfig = new Slot0Configs();
    intakePIDConfig.kP = Constants.Intake.kP; // 0.1
    intakePIDConfig.kI = Constants.Intake.kI; // 0.01;
    intakePIDConfig.kD = Constants.Intake.kD; // 0.0005;
    intakePIDConfig.kS = Constants.Intake.kS; // 0.10;
    intakePIDConfig.kV = Constants.Intake.kV; // 0.13;
    intakePIDConfig.kA = Constants.Intake.kA; // 3.0;
    intakeConfig.Slot0 = intakePIDConfig;

    m_intake.getConfigurator().apply(intakeConfig);

    m_intakeVelocity = m_intake.getVelocity();
  }

  /**
   * Commands the intake to spin at the given speed using closed-loop velocity control.
   *
   * <p>Converts the mechanism-level RPM to motor-shaft rotations per second via the gear
   * ratio, writes it to the velocity request object, and switches the control mode to
   * velocity control.
   *
   * @param rpm desired mechanism-side speed in RPM; positive spins in the configured positive direction
   */
  public void setIntakeRPM(double rpm) {
    m_intakeTargetRPM = rpm;
    m_intakeVelocityRequest.Velocity = rpm * Constants.Intake.intakeGearRatio / 60.0;
    m_intakeUseVelocityControl = true;
  }

  /**
   * Commands the intake motor using open-loop duty-cycle control.
   *
   * <p>Stores the target duty cycle, writes it to the duty-cycle request object, and
   * switches the control mode to open-loop.
   *
   * @param dutyCycle motor output fraction in the range [-1.0, 1.0]
   */
  public void setIntakePower(double dutyCycle) {
    m_intakeTargetDutyCycle = dutyCycle;
    m_intakelDutyCycleRequest.Output = dutyCycle;
    m_intakeUseVelocityControl = false;
  }

  /**
   * Stops the intake by commanding 0 RPM in velocity-control mode.
   *
   * <p>Delegates to {@link #setIntakeRPM(double)} with {@code 0}.
   */
  public void stop() {
    setIntakeRPM(0);
  }

  /**
   * Returns the current measured speed of the intake at the mechanism.
   *
   * <p>Converts the raw motor shaft velocity (rotations per second) to mechanism RPM
   * by applying the inverse of the gear ratio.
   *
   * @return current intake speed in RPM
   */
  public double getIntakeRPM() {
    return m_intakeVelocity.getValueAsDouble() * 60.0 / Constants.Intake.intakeGearRatio;
  }

  /**
   * Returns the most recently commanded target speed.
   *
   * @return target intake speed in RPM
   */
  public double getIntakeTargetRPM() {
    return m_intakeTargetRPM;
  }

  /**
   * Returns the difference between the target and measured intake speeds.
   *
   * @return RPM error (target - actual); positive when the intake is slower than desired
   */
  public double getIntakeRPMError() {
    return m_intakeTargetRPM - getIntakeRPM();
  }

  /**
   * Returns whether the intake has reached its target speed within tolerance.
   *
   * @return {@code true} if the absolute RPM error is less than {@link Constants.Intake#rpmTolerance}
   */
  public boolean isIntakeAtSpeed() {
    return Math.abs(getIntakeRPMError()) < Constants.Intake.rpmTolerance;
  }

  /**
   * Returns the most recently commanded open-loop duty cycle.
   *
   * @return target duty cycle in the range [-1.0, 1.0]
   */
  public double getIntakeTargetDutyCycle() {
    return m_intakeTargetDutyCycle;
  }

  /**
   * Periodic loop executed once per scheduler cycle (~20 ms).
   *
   * <p>Refreshes the cached velocity status signal, applies either the velocity or duty-cycle
   * control request depending on the active mode, and publishes comprehensive telemetry
   * to AdvantageKit {@link Logger}.
   */
  @Override
  public void periodic() {
    m_intakeVelocity.refresh();

    if (m_intakeUseVelocityControl) {
      m_intake.setControl(m_intakeVelocityRequest);
    } else {
      m_intake.setControl(m_intakelDutyCycleRequest);
    }

    Logger.recordOutput("Intake/Target RPM", m_intakeTargetRPM);
    Logger.recordOutput("Intake/Leader RPM", getIntakeRPM());
    Logger.recordOutput("Intake/RPM Error", getIntakeRPMError());
    Logger.recordOutput("Intake/At Speed", isIntakeAtSpeed());
    Logger.recordOutput("Intake/Target Duty Cycle", m_intakeTargetDutyCycle);
    Logger.recordOutput("Intake/Leader Voltage", m_intake.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Intake/Leader Stator Current", m_intake.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/Supply Current", m_intake.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/Torque Current", m_intake.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Intake/Closed Loop Error", m_intake.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput(
        "Intake/Closed Loop Output", m_intake.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput("Intake/Duty Cycle", m_intake.getDutyCycle().getValueAsDouble() * 1000);
    Logger.recordOutput("Intake/Leader Temp", m_intake.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Intake/Using Velocity Control", m_intakeUseVelocityControl);
  }
}