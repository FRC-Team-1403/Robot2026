package frc.robot.subsystems;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_flywheelLeader;
    private final TalonFX m_flywheelFollower;
    private final TalonFX m_flywheelFollower2;
    private final VelocityVoltage m_flywheelVelocityRequest;
    private final DutyCycleOut m_flywheelDutyCycleRequest;
    private double m_flywheelTargetRPM = 0;
    private double m_flywheelTargetDutyCycle = 0;
    private boolean m_flywheelUseVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_flywheelLeaderVelocity;
    private final StatusSignal<AngularVelocity> m_flywheelFollowerVelocity;
    private final StatusSignal<AngularVelocity> m_flywheelFollower2Velocity;

    public Shooter() {
        m_flywheelLeader = new TalonFX(Constants.Shooter.flywheelLeaderID);
        m_flywheelFollower = new TalonFX(Constants.Shooter.flywheelFollowerID);
        m_flywheelFollower2 = new TalonFX(Constants.Shooter.flywheelFollower2ID);
        m_flywheelVelocityRequest = new VelocityVoltage(0);
        m_flywheelVelocityRequest.Slot = 0;
        m_flywheelVelocityRequest.EnableFOC = true;
        m_flywheelDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
        flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit = 40;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

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
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 40;
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_flywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
        m_flywheelFollower.setControl(new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));

        TalonFXConfiguration flywheelFollower2Config = new TalonFXConfiguration();
        flywheelFollower2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelFollower2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelFollower2Config.CurrentLimits.StatorCurrentLimit = 40;
        flywheelFollower2Config.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelFollower2Config.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelFollower2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        m_flywheelTargetDutyCycle = 0;
        setFlywheelTargetRPM(0);
    }

    public double getFlywheelLeaderRPM() {
        return (m_flywheelLeaderVelocity.getValueAsDouble() * 60.0) / Constants.Shooter.flywheelGearRatio;
    }

    public double getFlywheelFollowerRPM() {
        return (m_flywheelFollowerVelocity.getValueAsDouble() * 60.0) / Constants.Shooter.flywheelGearRatio;
    }

    public double getFlywheelFollower2RPM() {
        return (m_flywheelFollower2Velocity.getValueAsDouble() * 60.0) / Constants.Shooter.flywheelGearRatio;
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

        SmartDashboard.putNumber("Flywheel/Target RPM", m_flywheelTargetRPM);
        SmartDashboard.putNumber("Flywheel/Leader RPM", getFlywheelLeaderRPM());
        SmartDashboard.putNumber("Flywheel/Follower RPM", getFlywheelFollowerRPM());
        SmartDashboard.putNumber("Flywheel/Follower2 RPM", getFlywheelFollower2RPM());
        SmartDashboard.putNumber("Flywheel/RPM Error", getFlywheelRPMError());
        SmartDashboard.putBoolean("Flywheel/At Speed", isFlywheelAtSpeed());
        SmartDashboard.putNumber("Flywheel/Target Duty Cycle", m_flywheelTargetDutyCycle);
        SmartDashboard.putNumber("Flywheel/Leader Voltage", m_flywheelLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Leader Stator Current", m_flywheelLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Follower Stator Current", m_flywheelFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Follower2 Stator Current", m_flywheelFollower2.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Supply Current", m_flywheelLeader.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Torque Current", m_flywheelLeader.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Closed Loop Error", m_flywheelLeader.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Closed Loop Output", m_flywheelLeader.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Duty Cycle", m_flywheelLeader.getDutyCycle().getValueAsDouble() * 1000);
        SmartDashboard.putNumber("Flywheel/Leader Temp", m_flywheelLeader.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Follower Temp", m_flywheelFollower.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Follower2 Temp", m_flywheelFollower2.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Flywheel/Using Velocity Control", m_flywheelUseVelocityControl);
    }
}