package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Indexer extends SubsystemBase {
  private final TalonFX m_indexerMotor;
  private final VelocityVoltage m_indexerVelocityRequest;
  private double m_indexerTargetRPM = 0;
  private final StatusSignal<AngularVelocity> m_indexerVelocity;

  public Indexer() {
    m_indexerMotor = new TalonFX(Constants.Indexer.m_indexerID,"Bus 2");

    m_indexerVelocityRequest = new VelocityVoltage(0);
    m_indexerVelocityRequest.Slot = 0;
    m_indexerVelocityRequest.EnableFOC = false;

    TalonFXConfiguration indexerLeaderConfig = new TalonFXConfiguration();
    indexerLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerLeaderConfig.CurrentLimits.StatorCurrentLimit = 120;
    indexerLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerLeaderConfig.CurrentLimits.SupplyCurrentLimit = 30;
    indexerLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerLeaderConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;
    indexerLeaderConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0; 

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

  public void setIndexerRPM(double rpm) {
    m_indexerTargetRPM = rpm;
    m_indexerVelocityRequest.Velocity = (rpm * Constants.Indexer.m_indexerGearRatio) / 60.0;
  }

  public void stop() {
    setIndexerRPM(0);
  }

  public double getIndexerRPM() {
    return (m_indexerVelocity.getValueAsDouble() * 60.0) / Constants.Indexer.m_indexerGearRatio;
  }

  public double getIndexerTargetRPM() {
    return m_indexerTargetRPM;
  }

  public double getIndexerRPMError() {
    return m_indexerTargetRPM - getIndexerRPM();
  }

  public boolean isIndexerAtSpeed() {
    return Math.abs(getIndexerRPMError()) < Constants.Indexer.rpmTolerance;
  }

  @Override
  public void periodic() {
    m_indexerVelocity.refresh();
    m_indexerMotor.setControl(m_indexerVelocityRequest);

    // Logger.recordOutput("Indexer/Target RPM", m_indexerTargetRPM);
    // Logger.recordOutput("Indexer/Leader RPM", getIndexerRPM());
    // Logger.recordOutput("Indexer/RPM Error", getIndexerRPMError());
    // Logger.recordOutput("Indexer/At Speed", isIndexerAtSpeed());
    // Logger.recordOutput(
    //     "Indexer/Leader Voltage", m_indexerMotor.getMotorVoltage().getValueAsDouble());
    // Logger.recordOutput(
    //     "Indexer/Leader Stator Current", m_indexerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Supply Current", m_indexerMotor.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Indexer/Torque Current", m_indexerMotor.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Indexer/Closed Loop Error", m_indexerMotor.getClosedLoopError().getValueAsDouble());
    // Logger.recordOutput(
    //     "Indexer/Closed Loop Output", m_indexerMotor.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput(
        "Indexer/Duty Cycle", m_indexerMotor.getDutyCycle().getValueAsDouble() * 1000);
    // Logger.recordOutput("Indexer/Leader Temp", m_indexerMotor.getDeviceTemp().getValueAsDouble());
  }
}
