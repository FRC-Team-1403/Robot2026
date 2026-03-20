package team1403.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import team1403.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase{
    private final TalonFX m_intake;
    private final TalonFX m_intakeFollower;
    private final VelocityVoltage m_intakeVelocityRequest;
    private final DutyCycleOut m_intakelDutyCycleRequest;
    private double m_intakeTargetRPM = 0;
    private double m_intakeTargetDutyCycle = 0;
    private boolean m_intakeUseVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_intakeVelocity;

     public Intake() {
        m_intake = new TalonFX(Constants.Intake.m_intakeTurretSideLeaderID, "Bus 2");
        m_intakeFollower = new TalonFX(Constants.Intake.m_intakeNonTurretSideFollowerID, "Bus 2");

        m_intakeVelocityRequest = new VelocityVoltage(0);
        m_intakeVelocityRequest.Slot = 0;
        m_intakeVelocityRequest.EnableFOC = false;
        m_intakelDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 120;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

        Slot0Configs intakePIDConfig = new Slot0Configs();
        intakePIDConfig.kP = Constants.Intake.kP;
        intakePIDConfig.kI = Constants.Intake.kI;
        intakePIDConfig.kD = Constants.Intake.kD;
        intakePIDConfig.kS = Constants.Intake.kS;
        intakePIDConfig.kV = Constants.Intake.kV;
        intakePIDConfig.kA = Constants.Intake.kA;
        intakeConfig.Slot0 = intakePIDConfig;

        TalonFXConfiguration intakeFollowerConfg = new TalonFXConfiguration();
        intakeFollowerConfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeFollowerConfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeFollowerConfg.CurrentLimits.StatorCurrentLimit = 120;
        intakeFollowerConfg.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeFollowerConfg.CurrentLimits.SupplyCurrentLimit = 70;
        intakeFollowerConfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeFollowerConfg.CurrentLimits.SupplyCurrentLowerLimit = 40;
        intakeFollowerConfg.CurrentLimits.SupplyCurrentLowerTime = 1.0;

        m_intakeFollower.getConfigurator().apply(intakeFollowerConfg);
        m_intakeFollower.setControl(new Follower(Constants.Intake.m_intakeTurretSideLeaderID, MotorAlignmentValue.Opposed));

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

        Logger.recordOutput("Intake/Target RPM", m_intakeTargetRPM);
        Logger.recordOutput("Intake/Leader RPM", getIntakeRPM());
        Logger.recordOutput("Intake/RPM Error", getIntakeRPMError());
        Logger.recordOutput("Intake/At Speed", isIntakeAtSpeed());
        Logger.recordOutput("Intake/Target Duty Cycle", m_intakeTargetDutyCycle);
        Logger.recordOutput("Intake/Leader Voltage", m_intake.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Intake/Leader Stator Current", m_intake.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Intake/Supply Current", m_intake.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Intake/Torque Current", m_intake.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Intake/Closed Loop Error", m_intake.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Intake/Closed Loop Output", m_intake.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Intake/Duty Cycle", m_intake.getDutyCycle().getValueAsDouble() * 1000);
        Logger.recordOutput("Intake/Leader Temp", m_intake.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Intake/Using Velocity Control", m_intakeUseVelocityControl);
        Logger.recordOutput("Intake/Follower Stator Current", m_intakeFollower.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Intake/Follower Duty Cycle", m_intakeFollower.getDutyCycle().getValueAsDouble() * 1000);
        Logger.recordOutput("Intake/Follower Voltage", m_intakeFollower.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Intake/Follower Supply Current", m_intakeFollower.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Intake/Follower Torque Current", m_intakeFollower.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Intake/Follower Temp", m_intakeFollower.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Intake/Follower RPM", m_intakeFollower.getVelocity().getValueAsDouble() * 60.0 / Constants.Intake.intakeGearRatio);

    }
}