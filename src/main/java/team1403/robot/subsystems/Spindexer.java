package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Spindexer extends SubsystemBase {
  private final TalonFX m_spindexerMotor;
  private final VelocityVoltage m_spindexerVelocityRequest;
  private double m_spindexerTargetRPM = 0;
  private final StatusSignal<AngularVelocity> m_spindexerVelocity;

  public Spindexer() {
    m_spindexerMotor = new TalonFX(Constants.Spindexer.m_spindexerID,"Bus 2");

    m_spindexerVelocityRequest = new VelocityVoltage(0);
    m_spindexerVelocityRequest.Slot = 0;
    m_spindexerVelocityRequest.EnableFOC = false;

    TalonFXConfiguration spindexerLeaderConfig = new TalonFXConfiguration();
    spindexerLeaderConfig.CurrentLimits.StatorCurrentLimit = 120;
    spindexerLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerLeaderConfig.CurrentLimits.SupplyCurrentLimit = 70;
    spindexerLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    spindexerLeaderConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    spindexerLeaderConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    spindexerLeaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


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

  public void setSpindexerRPM(double rpm) {
    m_spindexerTargetRPM = rpm;
    m_spindexerVelocityRequest.Velocity = (rpm * Constants.Spindexer.m_spindexerGearRatio) / 60.0;
  }

  public void stop() {
    setSpindexerRPM(0);
  }

  public double getSpindexerRPM() {
    return (m_spindexerVelocity.getValueAsDouble() * 60.0)
        / Constants.Spindexer.m_spindexerGearRatio;
  }

  public double getSpindexerTargetRPM() {
    return m_spindexerTargetRPM;
  }

  public double getSpindexerRPMError() {
    return m_spindexerTargetRPM - getSpindexerRPM();
  }

  public boolean isSpindexerAtSpeed() {
    return Math.abs(getSpindexerRPMError()) < Constants.Spindexer.rpmTolerance;
  }

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
