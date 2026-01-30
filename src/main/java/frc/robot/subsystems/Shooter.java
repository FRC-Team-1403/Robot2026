package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX m_motor;
  private final VelocityTorqueCurrentFOC m_velocityRequest;
  private final DutyCycleOut m_dutyCycleRequest;
  private double m_targetRPM = 0;
  private double m_targetDutyCycle = 0;
  private boolean m_useVelocityControl = true;

  private final StatusSignal<AngularVelocity> m_velocity;

  public Shooter() {
    m_motor = new TalonFX(1);
    m_velocityRequest = new VelocityTorqueCurrentFOC(0);
    m_dutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    var slot0 = config.Slot0;
    slot0.kS = 0.15;
    slot0.kV = 0.15;
    slot0.kA = 0.0;
    slot0.kP = 0.1;
    slot0.kI = 0.0;
    slot0.kD = 0.02;

    config.CurrentLimits.StatorCurrentLimit = 100;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.getConfigurator().apply(config);

    m_velocity = m_motor.getVelocity();
  }

  public void setTargetRPM(double rpm) {
    m_targetRPM = rpm;
    double rotationsPerSecond = rpm / 60.0;
    m_velocityRequest.Velocity = rotationsPerSecond;
    m_useVelocityControl = true;
  }

  public void setTargetPower(double dutyCycle) {
    m_targetDutyCycle = dutyCycle;
    m_dutyCycleRequest.Output = dutyCycle;
    m_useVelocityControl = false;
  }

  public void stop() {
    setTargetRPM(0);
  }

  public double getRPM() {
    return m_velocity.getValueAsDouble() * 60.0;
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public double getRPMError() {
    return m_targetRPM - getRPM();
  }

  public boolean isAtSpeed() {
    return Math.abs(getRPMError()) < Constants.Shooter.rpmTolerance;
  }

  public double getTargetDutyCycle() {
    return m_targetDutyCycle;
  }

  @Override
  public void periodic() {
    m_velocity.refresh();
    if (m_useVelocityControl) {
      m_motor.setControl(m_velocityRequest);
    } else {
      m_motor.setControl(m_dutyCycleRequest);
    }
    SmartDashboard.putNumber("Shooter/Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter/Actual RPM", getRPM());
    SmartDashboard.putNumber("Shooter/RPM Error", getRPMError());
    SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
    SmartDashboard.putNumber("Shooter/Target Duty Cycle", m_targetDutyCycle);
    SmartDashboard.putNumber("Shooter/Motor Voltage", m_motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Motor Current", m_motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble() * 1000);
    SmartDashboard.putNumber("Shooter/Motor Temp", m_motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter/Using Velocity Control", m_useVelocityControl);
  }
}
