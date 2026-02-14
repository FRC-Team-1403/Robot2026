package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class GroundIntake extends SubsystemBase {
    private final TalonFX m_flywheelLeader;
    private final VelocityVoltage m_flywheelVelocityRequest;
    private final DutyCycleOut m_flywheelDutyCycleRequest;
    private double m_flywheelTargetRPM = 0;
    private double m_flywheelTargetDutyCycle = 0;
    private boolean m_flywheelUseVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_flywheelLeaderVelocity;

    public GroundIntake() {
        m_flywheelLeader = new TalonFX(0);
        m_flywheelVelocityRequest = new VelocityVoltage(0);
        m_flywheelVelocityRequest.Slot = 0;
        m_flywheelVelocityRequest.EnableFOC = true;
        m_flywheelDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
        flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit = 40;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

        Slot0Configs flywheelPIDConfig = new Slot0Configs();
        flywheelPIDConfig.kP = 0.1;
        flywheelPIDConfig.kI = 0.01;
        flywheelPIDConfig.kD = 0.0005;
        flywheelPIDConfig.kS = 0.10;
        flywheelPIDConfig.kV = 0.13;
        flywheelPIDConfig.kA = 3.0;
        flywheelLeaderConfig.Slot0 = flywheelPIDConfig;

        m_flywheelLeader.getConfigurator().apply(flywheelLeaderConfig);

        m_flywheelLeaderVelocity = m_flywheelLeader.getVelocity();
    }

    public void setFlywheelTargetRPM(double rpm) {
        m_flywheelTargetRPM = rpm;
        m_flywheelVelocityRequest.Velocity = rpm * Constants.GroundIntake.intakeGearRatio / 60.0;
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
        return m_flywheelLeaderVelocity.getValueAsDouble() * 60.0 / Constants.GroundIntake.intakeGearRatio;
    }

    public double getFlywheelTargetRPM() {
        return m_flywheelTargetRPM;
    }

    public double getFlywheelRPMError() {
        return m_flywheelTargetRPM - getFlywheelLeaderRPM();
    }

    public boolean isFlywheelAtSpeed() {
        return Math.abs(getFlywheelRPMError()) < Constants.GroundIntake.rpmTolerance;
    }

    public double getFlywheelTargetDutyCycle() {
        return m_flywheelTargetDutyCycle;
    }

    @Override
    public void periodic() {
        m_flywheelLeaderVelocity.refresh();

        if (m_flywheelUseVelocityControl) {
            m_flywheelLeader.setControl(m_flywheelVelocityRequest);
        } else {
            m_flywheelLeader.setControl(m_flywheelDutyCycleRequest);
        }

        Logger.recordOutput("Flywheel/Target RPM", m_flywheelTargetRPM);
        Logger.recordOutput("Flywheel/Leader RPM", getFlywheelLeaderRPM());
        Logger.recordOutput("Flywheel/RPM Error", getFlywheelRPMError());
        Logger.recordOutput("Flywheel/At Speed", isFlywheelAtSpeed());
        Logger.recordOutput("Flywheel/Target Duty Cycle", m_flywheelTargetDutyCycle);
        Logger.recordOutput("Flywheel/Leader Voltage", m_flywheelLeader.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Flywheel/Leader Stator Current", m_flywheelLeader.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Flywheel/Supply Current", m_flywheelLeader.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Flywheel/Torque Current", m_flywheelLeader.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Flywheel/Closed Loop Error", m_flywheelLeader.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Flywheel/Closed Loop Output", m_flywheelLeader.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Flywheel/Duty Cycle", m_flywheelLeader.getDutyCycle().getValueAsDouble() * 1000);
        Logger.recordOutput("Flywheel/Leader Temp", m_flywheelLeader.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Flywheel/Using Velocity Control", m_flywheelUseVelocityControl);
    }
}
