package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX m_motor;
  private final VelocityVoltage m_velocityRequest;
  private final VoltageOut m_voltageRequest;
  private double m_targetRPM = 0;

  private final StatusSignal<AngularVelocity> m_velocity;
  private final SysIdRoutine m_sysIdRoutine;

  public Shooter() {
    m_motor = new TalonFX(1);
    m_velocityRequest = new VelocityVoltage(0);
    m_voltageRequest = new VoltageOut(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotionMagic.MotionMagicAcceleration = 400;
    
    var slot0 = config.Slot0;
    slot0.kS = 0.0;
    slot0.kV = 0.067;
    slot0.kA = 0.0;
    slot0.kP = 0.04;
    slot0.kI = 0.0;
    slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 50;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.getConfigurator().apply(config);

    m_velocity = m_motor.getVelocity();

    m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
        (state) -> Logger.recordOutput("SysIDShooter", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        m_motor.setVoltage(voltage.in(Volts));
      }, null, this));
  }
  
  public void setTargetRPM(double rpm) {
    m_targetRPM = rpm;
    double rotationsPerSecond = rpm / 60.0;
    m_velocityRequest.Velocity = rotationsPerSecond;
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

  public Command getSysIDQ(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.quasistatic(dir);
  }

  public Command getSysIDD(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.dynamic(dir);
  }

  @Override
  public void periodic() {
    m_velocity.refresh();
    m_motor.setControl(m_velocityRequest);
    SmartDashboard.putNumber("Shooter/Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter/Actual RPM", getRPM());
    SmartDashboard.putNumber("Shooter/RPM Error", getRPMError());
    SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
    SmartDashboard.putNumber("Shooter/Motor Voltage", m_motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Motor Current", m_motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble());
  }
}