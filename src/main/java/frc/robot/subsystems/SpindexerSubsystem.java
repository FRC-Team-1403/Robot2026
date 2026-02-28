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

public class SpindexerSubsystem extends SubsystemBase {
    private final TalonFX m_spindexer;
    private final VelocityVoltage m_spindexerVelocityRequest;
    private final DutyCycleOut m_spindexerDutyCycleRequest;
    private double m_spindexerTargetRPM = 0;
    private double m_spindexerTargetDutyCycle = 0;
    private boolean m_spindexerUseVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_spindexerVelocity;

    public SpindexerSubsystem() {
        m_spindexer = new TalonFX(1);
        m_spindexerVelocityRequest = new VelocityVoltage(0);
        m_spindexerVelocityRequest.Slot = 0;
        m_spindexerVelocityRequest.EnableFOC = false;
        m_spindexerDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
        spindexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = 40;
        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        Slot0Configs spindexerPIDConfig = new Slot0Configs();
        spindexerPIDConfig.kP = 0.04;
        spindexerPIDConfig.kI = 0.0;
        spindexerPIDConfig.kD = 0.01;
        spindexerPIDConfig.kS = 0.1;
        spindexerPIDConfig.kV = 0.115;
        spindexerPIDConfig.kA = 0.0;
        spindexerConfig.Slot0 = spindexerPIDConfig;

        m_spindexer.getConfigurator().apply(spindexerConfig);

        m_spindexerVelocity = m_spindexer.getVelocity();
    }

    public void setSpindexerTargetRPM(double rpm) {
        m_spindexerTargetRPM = rpm;
        m_spindexerVelocityRequest.Velocity = rpm * Constants.Spindexer.gearRatio / 60.0;
        m_spindexerUseVelocityControl = true;
    }

    public void setSpindexerTargetPower(double dutyCycle) {
        m_spindexerTargetDutyCycle = dutyCycle;
        m_spindexerDutyCycleRequest.Output = dutyCycle;
        m_spindexerUseVelocityControl = false;
    }

    public void stop() {
        setSpindexerTargetRPM(0);
    }

    public double getSpindexerRPM() {
        return m_spindexerVelocity.getValueAsDouble() * 60.0 / Constants.Spindexer.gearRatio;
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

    public double getSpindexerTargetDutyCycle() {
        return m_spindexerTargetDutyCycle;
    }

    @Override
    public void periodic() {
        m_spindexerVelocity.refresh();

        if (m_spindexerUseVelocityControl) {
            m_spindexer.setControl(m_spindexerVelocityRequest);
        } else {
            m_spindexer.setControl(m_spindexerDutyCycleRequest);
        }

        Logger.recordOutput("Spindexer/Target RPM", m_spindexerTargetRPM);
        Logger.recordOutput("Spindexer/RPM", getSpindexerRPM());
        Logger.recordOutput("Spindexer/RPM Error", getSpindexerRPMError());
        Logger.recordOutput("Spindexer/At Speed", isSpindexerAtSpeed());
        Logger.recordOutput("Spindexer/Target Duty Cycle", m_spindexerTargetDutyCycle);
        Logger.recordOutput("Spindexer/Voltage", m_spindexer.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Spindexer/Stator Current", m_spindexer.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Spindexer/Supply Current", m_spindexer.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Spindexer/Torque Current", m_spindexer.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Spindexer/Closed Loop Error", m_spindexer.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Spindexer/Closed Loop Output", m_spindexer.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Spindexer/Duty Cycle", m_spindexer.getDutyCycle().getValueAsDouble() * 1000);
        Logger.recordOutput("Spindexer/Temp", m_spindexer.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Spindexer/Using Velocity Control", m_spindexerUseVelocityControl);
    }
}