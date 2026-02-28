package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_indexer;
    private final VelocityVoltage m_indexerVelocityRequest;
    private final DutyCycleOut m_indexerDutyCycleRequest;
    private double m_indexerTargetRPM = 0;
    private double m_indexerTargetDutyCycle = 0;
    private boolean m_indexerUseVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_indexerVelocity;

    public IndexerSubsystem() {
        m_indexer = new TalonFX(2);
        m_indexerVelocityRequest = new VelocityVoltage(0);
        m_indexerVelocityRequest.Slot = 0;
        m_indexerVelocityRequest.EnableFOC = false;
        m_indexerDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerConfig.CurrentLimits.StatorCurrentLimit = 40;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        Slot0Configs indexerPIDConfig = new Slot0Configs();
        indexerPIDConfig.kP = 0.08;
        indexerPIDConfig.kI = 0.01;
        indexerPIDConfig.kD = 0.0005;
        indexerPIDConfig.kS = 0.10;
        indexerPIDConfig.kV = 0.12;
        indexerPIDConfig.kA = 3.0;
        indexerConfig.Slot0 = indexerPIDConfig;

        m_indexer.getConfigurator().apply(indexerConfig);

        m_indexerVelocity = m_indexer.getVelocity();
    }

    public void setIndexerTargetRPM(double rpm) {
        m_indexerTargetRPM = rpm;
        m_indexerVelocityRequest.Velocity = rpm * Constants.Indexer.gearRatio / 60.0;
        m_indexerUseVelocityControl = true;
    }

    public void setIndexerTargetPower(double dutyCycle) {
        m_indexerTargetDutyCycle = dutyCycle;
        m_indexerDutyCycleRequest.Output = dutyCycle;
        m_indexerUseVelocityControl = false;
    }

    public void stop() {
        setIndexerTargetRPM(0);
    }

    public double getIndexerRPM() {
        return m_indexerVelocity.getValueAsDouble() * 60.0 / Constants.Indexer.gearRatio;
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

    public double getIndexerTargetDutyCycle() {
        return m_indexerTargetDutyCycle;
    }

    @Override
    public void periodic() {
        m_indexerVelocity.refresh();

        if (m_indexerUseVelocityControl) {
            m_indexer.setControl(m_indexerVelocityRequest);
        } else {
            m_indexer.setControl(m_indexerDutyCycleRequest);
        }

        Logger.recordOutput("Indexer/Target RPM", m_indexerTargetRPM);
        Logger.recordOutput("Indexer/RPM", getIndexerRPM());
        Logger.recordOutput("Indexer/RPM Error", getIndexerRPMError());
        Logger.recordOutput("Indexer/At Speed", isIndexerAtSpeed());
        Logger.recordOutput("Indexer/Target Duty Cycle", m_indexerTargetDutyCycle);
        Logger.recordOutput("Indexer/Voltage", m_indexer.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Indexer/Stator Current", m_indexer.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Indexer/Supply Current", m_indexer.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Indexer/Torque Current", m_indexer.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Indexer/Closed Loop Error", m_indexer.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Indexer/Closed Loop Output", m_indexer.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Indexer/Duty Cycle", m_indexer.getDutyCycle().getValueAsDouble() * 1000);
        Logger.recordOutput("Indexer/Temp", m_indexer.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Indexer/Using Velocity Control", m_indexerUseVelocityControl);
    }
}