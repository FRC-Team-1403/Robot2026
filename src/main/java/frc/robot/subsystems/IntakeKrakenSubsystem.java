package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeKrakenSubsystem extends SubsystemBase {
    private TalonFX m_intakeMotor;
    private StatusSignal<AngularVelocity> m_velocity;
    private final PIDController m_pidController;
    private final SimpleMotorFeedforward m_feedforward;
    private boolean m_useVelocityControl = true;
    private double m_targetRPM = 0.0; //change

    public IntakeKrakenSubsystem() {
        m_intakeMotor = new TalonFX(0); //change?
        m_pidController = new PIDController(0.0005, 0.0, 0.0); //change
        m_feedforward = new SimpleMotorFeedforward(0.25, 0.12, 0.0); //change in terms of RPM

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_velocity = m_intakeMotor.getVelocity();

        m_intakeMotor.getConfigurator().apply(config);

    }

    public void setPercentPower(double percent) {
        m_intakeMotor.set(percent / 100);
    }

    public double getVelocityToRPM() {
        return m_velocity.getValue().in(edu.wpi.first.units.Units.RotationsPerSecond) * 60.0;
    }

    @Override
    public void periodic() {
        
        m_velocity.refresh();

        if (m_useVelocityControl) {
            
            double feedFowardVolts = m_feedforward.calculate(m_targetRPM);
            double PIDVolts = m_pidController.calculate(getVelocityToRPM(), m_targetRPM);

            m_intakeMotor.setVoltage(feedFowardVolts + PIDVolts);

        } else {
            m_intakeMotor.setControl(new DutyCycleOut(0.5)); //change
        }

        SmartDashboard.putNumber("Shooter/Target RPM", m_targetRPM);
        // SmartDashboard.putNumber("Shooter/Actual RPM", getRPM());
        // SmartDashboard.putNumber("Shooter/Motor 2 RPM", getRPM2());
        // SmartDashboard.putNumber("Shooter/RPM Error", getRPMError());
        // SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
        // SmartDashboard.putNumber("Shooter/Target Duty Cycle", m_targetDutyCycle);
        // SmartDashboard.putNumber("Shooter/Motor Voltage", m_motor.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Stator Current", m_motor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Motor 2 Stator Current", m_motor2.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Supply Current", m_motor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Torque Current", m_motor.getTorqueCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Closed Loop Error", m_motor.getClosedLoopError().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Closed Loop Output", m_motor.getClosedLoopOutput().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble() * 1000);
        // SmartDashboard.putNumber("Shooter/Motor Temp", m_motor.getDeviceTemp().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/Motor 2 Temp", m_motor2.getDeviceTemp().getValueAsDouble());
        // SmartDashboard.putBoolean("Shooter/Using Velocity Control", m_useVelocityControl);

    }

}
