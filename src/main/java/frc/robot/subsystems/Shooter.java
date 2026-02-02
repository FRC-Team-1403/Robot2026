package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_motor;
    private final TalonFX m_motor2;
    private final VelocityVoltage m_velocityRequest;
    private final DutyCycleOut m_dutyCycleRequest;
    private double m_targetRPM = 0;
    private double m_targetDutyCycle = 0;
    private boolean m_useVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final StatusSignal<AngularVelocity> m_velocity2;
    private final Slot0Configs slot0;

    public Shooter() {
        m_motor = new TalonFX(1);
        m_motor2 = new TalonFX(2);

        m_velocityRequest = new VelocityVoltage(0);
        m_velocityRequest.Slot = 0;
        m_velocityRequest.EnableFOC = true;
        m_dutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        slot0 = config.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.02;
        slot0.kP = 0.3;
        slot0.kI = 0.0;
        slot0.kD = 0.01;

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_motor.getConfigurator().apply(config);

        TalonFXConfiguration config2 = new TalonFXConfiguration();
        config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config2.CurrentLimits.StatorCurrentLimit = 40;
        config2.CurrentLimits.StatorCurrentLimitEnable = true;
        config2.CurrentLimits.SupplyCurrentLimit = 40;
        config2.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_motor2.getConfigurator().apply(config2);

        m_motor2.setControl(new Follower(1, MotorAlignmentValue.Opposed));

        m_velocity = m_motor.getVelocity();
        m_velocity2 = m_motor2.getVelocity();
    }

    public void setTargetRPM(double rpm) {
        m_targetRPM = rpm;
        m_velocityRequest.Velocity = rpm / 60.0;
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
        return m_velocity.getValue().in(edu.wpi.first.units.Units.RotationsPerSecond) * 60.0;
    }

    public double getRPM2() {
        return m_velocity2.getValue().in(edu.wpi.first.units.Units.RotationsPerSecond) * 60.0;
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
        m_velocity2.refresh();

        if (m_useVelocityControl) {
            m_motor.setControl(m_velocityRequest);
        } else {
            m_motor.setControl(m_dutyCycleRequest);
        }

        SmartDashboard.putNumber("Shooter/Target RPM", m_targetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", getRPM());
        SmartDashboard.putNumber("Shooter/Motor 2 RPM", getRPM2());
        SmartDashboard.putNumber("Shooter/RPM Error", getRPMError());
        SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
        SmartDashboard.putNumber("Shooter/Target Duty Cycle", m_targetDutyCycle);
        SmartDashboard.putNumber("Shooter/Motor Voltage", m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Stator Current", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor 2 Stator Current", m_motor2.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Supply Current", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Torque Current", m_motor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Closed Loop Error", m_motor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Closed Loop Output", m_motor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble() * 1000);
        SmartDashboard.putNumber("Shooter/Motor Temp", m_motor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor 2 Temp", m_motor2.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/Using Velocity Control", m_useVelocityControl);
        SmartDashboard.putNumber("Shooter/kP", slot0.kP);
    }
}
