package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CustomPositionControlLoop;

public class Turret extends SubsystemBase {
    private final TalonSRX m_motor;
    private final DutyCycleEncoder m_encoder;
    private final DigitalInput m_hallEffectSensor;
    private final CustomPositionControlLoop m_customController;

    private double currentAngle;
    private double setpoint;
    private double offset = 0.0;
    private final double magnetPosition = 0; 
    
    private boolean lastHallState = false;
    private boolean shouldReset = true;

    public Turret() {
        m_motor = new TalonSRX(Constants.TurretConstants.kTurretMotorID);
        m_motor.setInverted(Constants.TurretConstants.kMotorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);

        m_encoder = new DutyCycleEncoder(Constants.TurretConstants.kAbsEncoderPort);
        
        m_hallEffectSensor = new DigitalInput(Constants.TurretConstants.kHallEffectPort);

        m_customController = new CustomPositionControlLoop(
                Constants.TurretConstants.kGain,
                Constants.TurretConstants.kToleranceDegrees,
                Constants.TurretConstants.kRampUpTime,
                Constants.TurretConstants.kRampDownTime,
                1,
                Constants.TurretConstants.kMaxSpeed,
                Constants.TurretConstants.kMinSpeed,
                Constants.TurretConstants.kLoopTime
        );
        currentAngle = getTurretAngle();
        setpoint = currentAngle;
    }

    public double getTurretAngle() {
        return (m_encoder.get() + offset) * 360.0 * Constants.TurretConstants.gearRatio;
    }

    private void resetEncoder() {
        if (shouldReset) {
            offset = (magnetPosition / 360.0 / Constants.TurretConstants.gearRatio) - m_encoder.get();
            shouldReset = false;
        }
    }

    public void requestReset() {
        shouldReset = true;
    }

    public void setSetpoint(double degrees) {
        setpoint = degrees % 360.0;
        if (setpoint < 0) {
            setpoint += 360.0;
        }
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return m_customController.isAtPosition();
    }

    public void adjustSetpoint(double degrees) {
        setSetpoint(setpoint + degrees);
    }

    public void stopMotor() {
        m_motor.set(ControlMode.PercentOutput, 0.0);
        m_customController.reset();
    }
    
    private void setMotorOutput(double output) {
        m_motor.set(ControlMode.PercentOutput, output);
    }

    private double getShortestRotation(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }
        return error;
    }

    @Override
    public void periodic() {
        boolean hallState = !m_hallEffectSensor.get();
        if (hallState && !lastHallState) {
            requestReset();
        }
        lastHallState = hallState;
        resetEncoder();//only runs if requestReset() gets triggered

        currentAngle = getTurretAngle();

        double smallestError = getShortestRotation(setpoint, currentAngle);
        double goalSetpoint = currentAngle + smallestError;

        double motorOutput = m_customController.calculate(goalSetpoint, currentAngle);

        setMotorOutput(motorOutput);  
        
        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Setpoint", setpoint);
        SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Motor Output", motorOutput);  
        SmartDashboard.putNumber("Turret/P Value", m_customController.getP());
        SmartDashboard.putNumber("Turret/Position Error", smallestError);
        SmartDashboard.putBoolean("Turret/Hall Effect Triggered", hallState);
        SmartDashboard.putNumber("Turret/Encoder Offset", offset);
        SmartDashboard.putNumber("Turret/Raw Encoder", m_encoder.get());

        Logger.recordOutput("Turret/Current Angle", currentAngle);
        Logger.recordOutput("Turret/Setpoint", setpoint);
        Logger.recordOutput("Turret/At Setpoint", atSetpoint());
        Logger.recordOutput("Turret/Motor Output", motorOutput);
        Logger.recordOutput("Turret/P Value", m_customController.getP());
        Logger.recordOutput("Turret/Position Error", smallestError);
        Logger.recordOutput("Turret/Hall Effect Triggered", hallState);
        Logger.recordOutput("Turret/Encoder Offset", offset);
    }
}