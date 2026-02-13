package team1403.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team1403.robot.Constants;

public class GroundIntake extends SubsystemBase {
    private final TalonFX m_intakeMotor;
    private final ProfiledPIDController m_intakePIDController;
    private final SimpleMotorFeedforward m_intakeFeedforward;

    private double m_intakeTargetRPM = 0;
    private double m_intakeTargetDutyCycle = 0;

    private boolean m_intakeUseVelocityControl = true;

    private final VelocityTorqueCurrentFOC m_velocityRequest;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final DutyCycleOut m_dutyCycleRequest;

    public GroundIntake() {
        m_intakeMotor = new TalonFX(0);

        
        m_velocityRequest = new VelocityTorqueCurrentFOC(0);
        m_velocityRequest.Slot = 0;
 
        m_dutyCycleRequest = new DutyCycleOut();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 0.20;
        slot0.kI = 0.01;
        slot0.kD = 0.005;
        slot0.kS = 0.10;
        slot0.kV = 0.11;
        slot0.kA = 3.0;
        config.Slot0 = slot0;

        m_intakeMotor.getConfigurator().apply(config);

        TalonFXConfiguration config2 = new TalonFXConfiguration();
        config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config2.CurrentLimits.StatorCurrentLimit = 40;
        config2.CurrentLimits.StatorCurrentLimitEnable = true;
        config2.CurrentLimits.SupplyCurrentLimit = 40;
        config2.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_velocity = m_intakeMotor.getVelocity();

        m_intakePIDController = new ProfiledPIDController(
            Constants.GroundIntake.intakeKP,
            Constants.GroundIntake.intakeKI,
            Constants.GroundIntake.intakeKD,
            new TrapezoidProfile.Constraints(
                Constants.GroundIntake.intakeMaxVelocity,
                Constants.GroundIntake.intakeMaxAcceleration
            )
        );
        m_intakeFeedforward = new SimpleMotorFeedforward(
            Constants.GroundIntake.intakeKS,
            Constants.GroundIntake.intakeKV,
            Constants.GroundIntake.intakeKA
        );
    }

    public void setIntakeTargetRPM(double rpm) {
        m_intakeTargetRPM = rpm;
        m_velocityRequest.Velocity = rpm / 60.0;
        m_intakeUseVelocityControl = true;
    }

    public void setIntakeTargetPower(double dutyCycle) {
        m_intakeTargetDutyCycle = dutyCycle;
        m_dutyCycleRequest.Output = dutyCycle;
        m_intakeUseVelocityControl  = false;
    }

    public void stop() {
        setIntakeTargetRPM(0);
    }

    public double getIntakeRPM() {
        return m_velocity.getValue().in(edu.wpi.first.units.Units.RotationsPerSecond) * 60.0;
    }

    public double getIntakeTargetRPM() {
        return m_intakeTargetRPM;
    }

    public double getIntakeRPMError() {
        return m_intakeTargetRPM - getIntakeRPM();
    }

    public boolean isIntakeAtSpeed() {
        return m_intakePIDController.atGoal();
    }

    public double getIntakeTargetDutyCycle() {
        return m_intakeTargetDutyCycle;
    }

    @Override
    public void periodic() {
        if (m_intakeUseVelocityControl) {
            double motorTargetRPM = m_intakeTargetRPM * Constants.GroundIntake.intakeGearRatio;
            double feedforwardOutput = m_intakeFeedforward.calculate(motorTargetRPM);
            double pidOutput = m_intakePIDController.calculate(getIntakeRPM(), m_intakeTargetRPM);
            m_intakeMotor.setVoltage(feedforwardOutput + pidOutput);
        } else {
            m_intakeMotor.set(m_intakeTargetDutyCycle);
        }

        SmartDashboard.putNumber("Intake/RPM", getIntakeRPM());
        SmartDashboard.putNumber("Intake/Target RPM", m_intakeTargetRPM);
        SmartDashboard.putNumber("Intake/RPM Error", getIntakeRPMError());
        SmartDashboard.putBoolean("Intake/At Speed", isIntakeAtSpeed());
        SmartDashboard.putNumber("Intake/Target Duty Cycle", m_intakeTargetDutyCycle);
        //SmartDashboard.putNumber("Intake/Voltage", m_intakeMotor.getAppliedOutput() * m_intakeMotor.getBusVoltage());
        //SmartDashboard.putNumber("Intake/Current", m_intakeMotor.getOutputCurrent());
        //SmartDashboard.putNumber("Intake/Temp", m_intakeMotor.getMotorTemperature());
        //SmartDashboard.putBoolean("Intake/Using Velocity Control", m_intakeUseVelocityControl);
    }
}