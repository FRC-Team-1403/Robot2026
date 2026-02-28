package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private final TalonFX m_intake;
    private final VelocityVoltage m_intakeVelocityRequest;
    private final DutyCycleOut m_intakelDutyCycleRequest;
    private double m_intakeTargetRPM = 0;
    private double m_intakeTargetDutyCycle = 0;
    private boolean m_intakeUseVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_intakeVelocity;

     public Intake() {
        m_intake = new TalonFX(Constants.Intake.m_intakeID);

        m_intakeVelocityRequest = new VelocityVoltage(0);
        m_intakeVelocityRequest.Slot = 0;
        m_intakelDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 40;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

        Slot0Configs intakePIDConfig = new Slot0Configs();
        intakePIDConfig.kP = Constants.Intake.kP;//0.1
        intakePIDConfig.kI = Constants.Intake.kI;//0.01;
        intakePIDConfig.kD = Constants.Intake.kD;//0.0005;
        intakePIDConfig.kS = Constants.Intake.kS;//0.10;
        intakePIDConfig.kV = Constants.Intake.kV;//0.13;
        intakePIDConfig.kA = Constants.Intake.kA;//3.0;
        intakeConfig.Slot0 = intakePIDConfig;

        m_intake.getConfigurator().apply(intakeConfig);

        m_intakeVelocity = m_intake.getVelocity();
    }

    public void setIntakeRPM(double rpm) {
        m_intakeTargetRPM = rpm;
        m_intakeVelocityRequest.Velocity = rpm * Constants.Intake.intakeGearRatio / 60.0;
        m_intakeUseVelocityControl = true;
    }

    public void setIntakePower(double dutyCycle) {
        m_intakeTargetDutyCycle = dutyCycle;
        m_intakelDutyCycleRequest.Output = dutyCycle;
        m_intakeUseVelocityControl = false;
    }

    public void stop() {
        setIntakeRPM(0);
    }

    public double getIntakeRPM() {
        return m_intakeVelocity.getValueAsDouble() * 60.0 / Constants.Intake.intakeGearRatio;
    }

    public double getIntakeTargetRPM() {
        return m_intakeTargetRPM;
    }

    public double getIntakeRPMError() {
        return m_intakeTargetRPM - getIntakeRPM();
    }

    public boolean isIntakeAtSpeed() {
        return Math.abs(getIntakeRPMError()) < Constants.Intake.rpmTolerance;
    }

    public double getIntakeTargetDutyCycle() {
        return m_intakeTargetDutyCycle;
    }

    @Override
    public void periodic() {
        m_intakeVelocity.refresh();

        if (m_intakeUseVelocityControl) {
            m_intake.setControl(m_intakeVelocityRequest);
        } else {
            m_intake.setControl(m_intakelDutyCycleRequest);
        }

        SmartDashboard.putNumber("Intake/Target RPM", m_intakeTargetRPM);
        SmartDashboard.putNumber("Intake/Leader RPM", getIntakeRPM());
        SmartDashboard.putNumber("Intake/RPM Error", getIntakeRPMError());
        SmartDashboard.putBoolean("Intake/At Speed", isIntakeAtSpeed());
        SmartDashboard.putNumber("Intake/Target Duty Cycle", m_intakeTargetDutyCycle);
        SmartDashboard.putNumber("Intake/Leader Voltage", m_intake.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Leader Stator Current", m_intake.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Supply Current", m_intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Torque Current", m_intake.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Closed Loop Error", m_intake.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Closed Loop Output", m_intake.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Duty Cycle", m_intake.getDutyCycle().getValueAsDouble() * 1000);
        SmartDashboard.putNumber("Intake/Leader Temp", m_intake.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Using Velocity Control", m_intakeUseVelocityControl);
    }

}