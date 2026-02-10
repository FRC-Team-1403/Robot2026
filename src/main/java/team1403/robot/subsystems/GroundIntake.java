package team1403.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team1403.robot.Constants;

public class GroundIntake extends SubsystemBase {
    private final SparkMax m_intakeMotor;
    private final ProfiledPIDController m_intakePIDController;
    private final SimpleMotorFeedforward m_intakeFeedforward;
    private final RelativeEncoder m_intakeEncoder;

    private double m_intakeTargetRPM = 0;
    private double m_intakeTargetDutyCycle = 0;

    private boolean m_intakeUseVelocityControl = true;

    public GroundIntake() {
        m_intakeMotor = new SparkMax(Constants.GroundIntake.intakeMotorID, MotorType.kBrushless);
        m_intakeEncoder = m_intakeMotor.getEncoder();

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

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.smartCurrentLimit((int) Constants.GroundIntake.intakeCurrentLimit);
        intakeConfig.inverted(false);
        m_intakeMotor.configure(
            intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }

    public void setIntakeTargetRPM(double rpm) {
        m_intakeTargetRPM = rpm;
        m_intakeUseVelocityControl = true;
    }

    public void setIntakeTargetPower(double dutyCycle) {
        m_intakeTargetDutyCycle = dutyCycle;
        m_intakeUseVelocityControl = false;
    }

    public void stop() {
        setIntakeTargetRPM(0);
    }

    public double getIntakeRPM() {
        return m_intakeEncoder.getVelocity() / Constants.GroundIntake.intakeGearRatio;
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
        SmartDashboard.putNumber("Intake/Voltage", m_intakeMotor.getAppliedOutput() * m_intakeMotor.getBusVoltage());
        SmartDashboard.putNumber("Intake/Current", m_intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Temp", m_intakeMotor.getMotorTemperature());
        //SmartDashboard.putBoolean("Intake/Using Velocity Control", m_intakeUseVelocityControl);
    }
}