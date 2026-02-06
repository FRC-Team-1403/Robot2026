package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_flywheelLeader;
    private final TalonFX m_flywheelFollower;
    private final SparkMax m_rollerMotor;
    private final ProfiledPIDController m_rollerPIDController;
    private final SimpleMotorFeedforward m_rollerFeedforward;
    private final RelativeEncoder m_rollerEncoder;

    private final VelocityTorqueCurrentFOC m_flywheelVelocityRequest;
    private final DutyCycleOut m_flywheelDutyCycleRequest;

    private double m_flywheelTargetRPM = 0;
    private double m_flywheelTargetDutyCycle = 0;
    private double m_rollerTargetRPM = 0;
    private double m_rollerTargetDutyCycle = 0;

    private boolean m_flywheelUseVelocityControl = true;
    private boolean m_rollerUseVelocityControl = true;

    private final StatusSignal<AngularVelocity> m_flywheelLeaderVelocity;
    private final StatusSignal<AngularVelocity> m_flywheelFollowerVelocity;

    public Shooter() {
        m_flywheelLeader = new TalonFX(Constants.Shooter.flywheelLeaderID);
        m_flywheelFollower = new TalonFX(Constants.Shooter.flywheelFollowerID);
        m_rollerMotor = new SparkMax(Constants.Shooter.rollerMotorID, MotorType.kBrushless);
        m_rollerEncoder = m_rollerMotor.getEncoder();

        m_rollerPIDController = new ProfiledPIDController(
            Constants.Shooter.rollerKP,
            Constants.Shooter.rollerKI,
            Constants.Shooter.rollerKD,
            new TrapezoidProfile.Constraints(
                Constants.Shooter.rollerMaxVelocity,
                Constants.Shooter.rollerMaxAcceleration
            )
        );
        m_rollerFeedforward = new SimpleMotorFeedforward(
            Constants.Shooter.rollerKS,
            Constants.Shooter.rollerKV,
            Constants.Shooter.rollerKA
        );

        m_flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0);
        m_flywheelVelocityRequest.Slot = 0;
        m_flywheelVelocityRequest.FeedForward = 0;

        m_flywheelDutyCycleRequest = new DutyCycleOut(0);

        TalonFXConfiguration flywheelLeaderConfig = new TalonFXConfiguration();
        flywheelLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.flywheelStatorCurrentLimit;
        flywheelLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.flywheelSupplyCurrentLimit;
        flywheelLeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        Slot0Configs flywheelPIDConfig = new Slot0Configs();
        flywheelPIDConfig.kP = Constants.Shooter.flywheelKP;
        flywheelPIDConfig.kI = Constants.Shooter.flywheelKI;
        flywheelPIDConfig.kD = Constants.Shooter.flywheelKD;
        flywheelPIDConfig.kS = Constants.Shooter.flywheelKS;
        flywheelPIDConfig.kV = Constants.Shooter.flywheelKV;
        flywheelPIDConfig.kA = Constants.Shooter.flywheelKA;
        flywheelLeaderConfig.Slot0 = flywheelPIDConfig;

        m_flywheelLeader.getConfigurator().apply(flywheelLeaderConfig);

        TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
        flywheelFollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.flywheelStatorCurrentLimit;
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.flywheelSupplyCurrentLimit;
        flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_flywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
        m_flywheelFollower.setControl(new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit((int) Constants.Shooter.rollerCurrentLimit);
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
        m_flywheelVelocityRequest.Velocity = rpm / 60.0;
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
        return m_flywheelLeaderVelocity.getValueAsDouble() * 60.0;
    }

    public double getFlywheelFollowerRPM() {
        return m_flywheelFollowerVelocity.getValueAsDouble() * 60.0;
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
            double motorTargetRPM = m_rollerTargetRPM * Constants.Shooter.rollerGearRatio;
            double feedforwardOutput = m_rollerFeedforward.calculate(motorTargetRPM);
            double pidOutput = m_rollerPIDController.calculate(getRollerRPM(), m_rollerTargetRPM);
            m_rollerMotor.setVoltage(feedforwardOutput + pidOutput);
        } else {
            m_rollerMotor.set(m_rollerTargetDutyCycle);
        }

        SmartDashboard.putNumber("Flywheel/Target RPM", m_flywheelTargetRPM);
        SmartDashboard.putNumber("Flywheel/Leader RPM", getFlywheelLeaderRPM());
        SmartDashboard.putNumber("Flywheel/Follower RPM", getFlywheelFollowerRPM());
        SmartDashboard.putNumber("Flywheel/RPM Error", getFlywheelRPMError());
        SmartDashboard.putBoolean("Flywheel/At Speed", isFlywheelAtSpeed());
        SmartDashboard.putNumber("Flywheel/Target Duty Cycle", m_flywheelTargetDutyCycle);
        SmartDashboard.putNumber("Flywheel/Leader Voltage", m_flywheelLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Leader Stator Current", m_flywheelLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Follower Stator Current", m_flywheelFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Supply Current", m_flywheelLeader.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Torque Current", m_flywheelLeader.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Feedforward Current", m_flywheelVelocityRequest.FeedForward);
        SmartDashboard.putNumber("Flywheel/Closed Loop Error", m_flywheelLeader.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Closed Loop Output", m_flywheelLeader.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Duty Cycle", m_flywheelLeader.getDutyCycle().getValueAsDouble() * 1000);
        SmartDashboard.putNumber("Flywheel/Leader Temp", m_flywheelLeader.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Follower Temp", m_flywheelFollower.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Flywheel/Using Velocity Control", m_flywheelUseVelocityControl);

        SmartDashboard.putNumber("Roller/RPM", getRollerRPM());
        SmartDashboard.putNumber("Roller/Target RPM", m_rollerTargetRPM);
        SmartDashboard.putNumber("Roller/RPM Error", getRollerRPMError());
        SmartDashboard.putBoolean("Roller/At Speed", isRollerAtSpeed());
        SmartDashboard.putNumber("Roller/Target Duty Cycle", m_rollerTargetDutyCycle);
        SmartDashboard.putNumber("Roller/Voltage", m_rollerMotor.getAppliedOutput() * m_rollerMotor.getBusVoltage());
        SmartDashboard.putNumber("Roller/Current", m_rollerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Roller/Temp", m_rollerMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Roller/Using Velocity Control", m_rollerUseVelocityControl);
    }
}
