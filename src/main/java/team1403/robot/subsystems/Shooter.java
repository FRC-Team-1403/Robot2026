package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
import team1403.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX m_flywheelLeader;
  private final TalonFX m_flywheelFollower;
  private final TalonFX m_flywheelFollower2;
  private final VelocityVoltage m_flywheelVelocityRequest;
  private final DutyCycleOut m_flywheelDutyCycleRequest;
  private double m_flywheelTargetRPM = 0;
  private double m_flywheelTargetDutyCycle = 0;
  private boolean m_flywheelUseVelocityControl = true;

  @SuppressWarnings("all")
  private final StatusSignal m_flywheelLeaderVelocity;

  @SuppressWarnings("all")
  private final StatusSignal m_flywheelFollowerVelocity;

  @SuppressWarnings("all")
  private final StatusSignal m_flywheelFollower2Velocity;

  public Shooter() {
    m_flywheelLeader = new TalonFX(Constants.Shooter.flywheelLeaderID,"Bus 2");
    m_flywheelFollower = new TalonFX(Constants.Shooter.flywheelFollower1TopLeftID,"Bus 2");
    m_flywheelFollower2 = new TalonFX(Constants.Shooter.flywheelFollower2BottonLeftID, "Bus 2");
    m_flywheelVelocityRequest = new VelocityVoltage(0);
    m_flywheelVelocityRequest.Slot = 0;
    m_flywheelVelocityRequest.EnableFOC = false;
    m_flywheelDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
    flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit = 120;
    flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit = 60;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    flywheelLeaderConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;

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
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 120;
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;

    m_flywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
    
    m_flywheelFollower.setControl(new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));

    TalonFXConfiguration flywheelFollower2Config = new TalonFXConfiguration();
    flywheelFollower2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelFollower2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelFollower2Config.CurrentLimits.StatorCurrentLimit = 120;
    flywheelFollower2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelFollower2Config.CurrentLimits.SupplyCurrentLimit = 60;
    flywheelFollower2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelFollower2Config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    flywheelFollower2Config.CurrentLimits.SupplyCurrentLowerTime = 2.0;
    
    m_flywheelFollower2.getConfigurator().apply(flywheelFollower2Config);
    m_flywheelFollower2.setControl(new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));

    m_flywheelLeaderVelocity = m_flywheelLeader.getVelocity();
    m_flywheelFollowerVelocity = m_flywheelFollower.getVelocity();
    m_flywheelFollower2Velocity = m_flywheelFollower2.getVelocity();
  }

  public void setFlywheelTargetRPM(double rpm) {
    m_flywheelTargetRPM = rpm;
    m_flywheelVelocityRequest.Velocity = (rpm * Constants.Shooter.flywheelGearRatio) / 60.0;
    m_flywheelUseVelocityControl = true;
  }

  public void setFlywheelTargetPower(double dutyCycle) {
    m_flywheelTargetDutyCycle = dutyCycle;
    m_flywheelDutyCycleRequest.Output = dutyCycle;
    m_flywheelUseVelocityControl = false;
  }

  public void stop() {
    setFlywheelTargetRPM(0);
  }

  public double getFlywheelLeaderRPM() {
    return (m_flywheelLeaderVelocity.getValueAsDouble() * 60.0)
        / Constants.Shooter.flywheelGearRatio;
  }

  public double getFlywheelFollowerRPM() {
    return (m_flywheelFollowerVelocity.getValueAsDouble() * 60.0)
        / Constants.Shooter.flywheelGearRatio;
  }

  public double getFlywheelFollower2RPM() {
    return (m_flywheelFollower2Velocity.getValueAsDouble() * 60.0)
        / Constants.Shooter.flywheelGearRatio;
  }

  public double getFlywheelTargetRPM() {
    return m_flywheelTargetRPM;
  }

  public double getFlywheelRPMError() {
    return m_flywheelTargetRPM - getFlywheelLeaderRPM();
  }

  public boolean isFlywheelAtSpeed() {
    return Math.abs(getFlywheelRPMError()) < Constants.Shooter.rpmTolerance;
  }

  public double getFlywheelTargetDutyCycle() {
    return m_flywheelTargetDutyCycle;
  }

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
