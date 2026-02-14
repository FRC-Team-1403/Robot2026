package frc.robot.subsystems;

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
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_flywheelLeader;
    private final TalonFX m_flywheelFollower;
    private final SparkMax m_rollerMotor;
    private final ProfiledPIDController m_rollerPIDController;
    private final SimpleMotorFeedforward m_rollerFeedforward;
    private final RelativeEncoder m_rollerEncoder;
    private final VelocityVoltage m_flywheelVelocityRequest;
    private final DutyCycleOut m_flywheelDutyCycleRequest;
    private final SlewRateLimiter m_flywheelSlewLimiter;
    private final SlewRateLimiter m_rollerSlewLimiter;
    private double m_flywheelTargetRPM = 0;
    private double m_flywheelTargetDutyCycle = 0;
    private double m_rollerTargetRPM = 0;
    private double m_rollerTargetDutyCycle = 0;
    private boolean m_flywheelUseVelocityControl = true;
    private boolean m_rollerUseVelocityControl = true;
    private final StatusSignal<AngularVelocity> m_flywheelLeaderVelocity;
    private final StatusSignal<AngularVelocity> m_flywheelFollowerVelocity;

    public Shooter() {
        m_flywheelLeader = new TalonFX(0);
        m_flywheelFollower = new TalonFX(1);
        m_rollerMotor = new SparkMax(42, MotorType.kBrushless);
        m_rollerEncoder = m_rollerMotor.getEncoder();
        m_rollerPIDController = new ProfiledPIDController(
                0.00004,
                0,
                0.0001,
                new TrapezoidProfile.Constraints(5000, 10000));
        m_rollerFeedforward = new SimpleMotorFeedforward(0.0005, 0.00205, 0.0003);
        m_flywheelVelocityRequest = new VelocityVoltage(0);
        m_flywheelVelocityRequest.Slot = 0;
        m_flywheelVelocityRequest.EnableFOC = true;
        m_flywheelDutyCycleRequest = new DutyCycleOut(0);
        m_flywheelSlewLimiter = new SlewRateLimiter(4000.0 / 60.0 * Constants.Shooter.flywheelGearRatio);
        m_rollerSlewLimiter = new SlewRateLimiter(4000.0 / Constants.Shooter.rollerGearRatio);

        TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
        flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit = 40;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

        Slot0Configs flywheelPIDConfig = new Slot0Configs();
        flywheelPIDConfig.kP = 0.08;
        flywheelPIDConfig.kI = 0.01;
        flywheelPIDConfig.kD = 0.0005;
        flywheelPIDConfig.kS = 0.10;
        flywheelPIDConfig.kV = 0.123;
        flywheelPIDConfig.kA = 3.0;
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
        m_flywheelFollower.setControl(new Follower(0, MotorAlignmentValue.Opposed));

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(40);
        rollerConfig.inverted(true);
        m_rollerMotor.configure(
                rollerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );

        m_flywheelLeaderVelocity = m_flywheelLeader.getVelocity();
        m_flywheelFollowerVelocity = m_flywheelFollower.getVelocity();
    }

    public void setFlywheelTargetRPM(double rpm) {
        m_flywheelTargetRPM = rpm;
        m_flywheelVelocityRequest.Velocity = m_flywheelSlewLimiter.calculate(rpm * Constants.Shooter.flywheelGearRatio / 60.0);
        m_flywheelUseVelocityControl = true;
    }

    public void setFlywheelTargetPower(double dutyCycle) {
        m_flywheelTargetDutyCycle = dutyCycle;
        m_flywheelDutyCycleRequest.Output = dutyCycle;
        m_flywheelUseVelocityControl = false;
    }

    public void setRollerTargetRPM(double rpm) {
        m_rollerTargetRPM = rpm;
        m_rollerUseVelocityControl = true;
    }

    public void setRollerTargetPower(double dutyCycle) {
        m_rollerTargetDutyCycle = dutyCycle;
        m_rollerUseVelocityControl = false;
    }

    public void stop() {
        setFlywheelTargetRPM(0);
        setRollerTargetRPM(0);
    }

    public double getFlywheelLeaderRPM() {
        return m_flywheelLeaderVelocity.getValueAsDouble() * 60.0 / Constants.Shooter.flywheelGearRatio;
    }

    public double getFlywheelFollowerRPM() {
        return m_flywheelFollowerVelocity.getValueAsDouble() * 60.0 / Constants.Shooter.flywheelGearRatio;
    }

    public double getRollerRPM() {
        return m_rollerEncoder.getVelocity() / Constants.Shooter.rollerGearRatio;
    }

    public double getFlywheelTargetRPM() {
        return m_flywheelTargetRPM;
    }

    public double getRollerTargetRPM() {
        return m_rollerTargetRPM;
    }

    public double getFlywheelRPMError() {
        return m_flywheelTargetRPM - getFlywheelLeaderRPM();
    }

    public double getRollerRPMError() {
        return m_rollerTargetRPM - getRollerRPM();
    }

    public boolean isFlywheelAtSpeed() {
        return Math.abs(getFlywheelRPMError()) < Constants.Shooter.rpmTolerance;
    }

    public boolean isRollerAtSpeed() {
        return m_rollerPIDController.atGoal();
    }

    public double getFlywheelTargetDutyCycle() {
        return m_flywheelTargetDutyCycle;
    }

    public double getRollerTargetDutyCycle() {
        return m_rollerTargetDutyCycle;
    }

    @Override
    public void periodic() {
        m_flywheelLeaderVelocity.refresh();
        m_flywheelFollowerVelocity.refresh();

        if (m_flywheelUseVelocityControl) {
            m_flywheelLeader.setControl(m_flywheelVelocityRequest);
        } else {
            m_flywheelLeader.setControl(m_flywheelDutyCycleRequest);
        }

        if (m_rollerUseVelocityControl) {
            double motorTargetRPM = m_rollerSlewLimiter.calculate(m_rollerTargetRPM * Constants.Shooter.rollerGearRatio);
            double feedforwardOutput = m_rollerFeedforward.calculate(motorTargetRPM);
            double pidOutput = m_rollerPIDController.calculate(getRollerRPM(), m_rollerTargetRPM);
            m_rollerMotor.setVoltage(feedforwardOutput + pidOutput);
        } else {
            m_rollerMotor.set(m_rollerTargetDutyCycle);
        }

        Logger.recordOutput("Flywheel/Target RPM", m_flywheelTargetRPM);
        Logger.recordOutput("Flywheel/Leader RPM", getFlywheelLeaderRPM());
        Logger.recordOutput("Flywheel/Follower RPM", getFlywheelFollowerRPM());
        Logger.recordOutput("Flywheel/RPM Error", getFlywheelRPMError());
        Logger.recordOutput("Flywheel/At Speed", isFlywheelAtSpeed());
        Logger.recordOutput("Flywheel/Target Duty Cycle", m_flywheelTargetDutyCycle);
        Logger.recordOutput("Flywheel/Leader Voltage", m_flywheelLeader.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Flywheel/Leader Stator Current", m_flywheelLeader.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Flywheel/Follower Stator Current", m_flywheelFollower.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Flywheel/Supply Current", m_flywheelLeader.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Flywheel/Torque Current", m_flywheelLeader.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("Flywheel/Closed Loop Error", m_flywheelLeader.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Flywheel/Closed Loop Output", m_flywheelLeader.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Flywheel/Duty Cycle", m_flywheelLeader.getDutyCycle().getValueAsDouble() * 1000);
        Logger.recordOutput("Flywheel/Leader Temp", m_flywheelLeader.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Flywheel/Follower Temp", m_flywheelFollower.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Flywheel/Using Velocity Control", m_flywheelUseVelocityControl);
        Logger.recordOutput("Roller/RPM", getRollerRPM());
        Logger.recordOutput("Roller/Target RPM", m_rollerTargetRPM);
        Logger.recordOutput("Roller/RPM Error", getRollerRPMError());
        Logger.recordOutput("Roller/At Speed", isRollerAtSpeed());
        Logger.recordOutput("Roller/Target Duty Cycle", m_rollerTargetDutyCycle);
        Logger.recordOutput("Roller/Voltage", m_rollerMotor.getAppliedOutput() * m_rollerMotor.getBusVoltage());
        Logger.recordOutput("Roller/Current", m_rollerMotor.getOutputCurrent());
        Logger.recordOutput("Roller/Temp", m_rollerMotor.getMotorTemperature());
        Logger.recordOutput("Roller/Using Velocity Control", m_rollerUseVelocityControl);
    }
}
