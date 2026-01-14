package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.TurretEncoder;

public class Turret extends SubsystemBase {

    private final TalonSRX m_motor;
    private final TurretEncoder m_encoder;

    public Turret() {
        m_motor = new TalonSRX(Constants.TurretConstants.kTurretMotorID);
        m_motor.setInverted(Constants.TurretConstants.kMotorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);

        m_encoder = new TurretEncoder(Constants.TurretConstants.kAbsEncoderPort);
    }

    public double getTurretAngle() {
        return m_encoder.getTurretAngle();
    }

    public void setSetpoint(double degrees) {
        m_encoder.setSetpoint(degrees);
    }

    public double getSetpoint() {
        return m_encoder.getSetpoint();
    }

    public boolean atSetpoint() {
        return m_encoder.atSetpoint();
    }

    public void adjustSetpoint(double degrees) {
        setSetpoint(m_encoder.getSetpoint() + degrees);
    }

    public void stopMotor() {
        m_motor.set(ControlMode.PercentOutput, 0.0);
        m_encoder.reset();
    }

    public void setSpeedManual(double speed) {
        if(speed > 0.5) {
            speed = 0.5;
        }
        if(speed < -0.5) {
            speed = -0.5;
        }
        
        double angle = getTurretAngle();
        
        if(angle < 360.0 - m_encoder.getDeadzoneWidth() && angle > 0) {
            m_motor.set(ControlMode.PercentOutput, speed);
        } else if(angle > 360 - m_encoder.getDeadzoneWidth() && speed < 0) {
            m_motor.set(ControlMode.PercentOutput, speed);
        } else if(angle < 0 && speed > 0) {
            m_motor.set(ControlMode.PercentOutput, speed);
        } else {
            stopMotor();
        }
    }

    private void setMotorOutput(double output) {
        m_motor.set(ControlMode.PercentOutput, output);
    }

    public TalonSRX getMotor() {
        return m_motor;
    }

    public TurretEncoder getEncoder() {
        return m_encoder;
    }

    @Override
    public void periodic() {
        double motorOutput = m_encoder.getControllerOutput();
        
        setMotorOutput(motorOutput / 100.0);

        SmartDashboard.putNumber("Turret/Current Angle", getTurretAngle());
        SmartDashboard.putNumber("Turret/Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Motor Output", motorOutput / 100.0);
        SmartDashboard.putNumber("Turret/P Value", m_encoder.getP());
        SmartDashboard.putNumber("Turret/Raw Angle", m_encoder.getRaw());
        SmartDashboard.putNumber("Turret/Raw Angle With Gear Ratio", m_encoder.getRawWithGearRatio());
        SmartDashboard.putBoolean("Turret/Encoder Connected", m_encoder.isConnected());
    }
}