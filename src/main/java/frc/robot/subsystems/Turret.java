package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

public class Turret extends SubsystemBase {
    private final TalonSRX m_motor;
    private final DutyCycleEncoder m_encoder;
    private final CustomPositionControlLoop m_customController;
    
    private double currentAngle;
    private double setpoint;
    
    private static final double LIMIT_BUFFER = 2.0;

    public Turret() {
        m_motor = new TalonSRX(Constants.TurretConstants.kTurretMotorID);
        m_motor.setInverted(Constants.TurretConstants.kMotorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);
        
        m_encoder = new DutyCycleEncoder(Constants.TurretConstants.kAbsEncoderPort);
        
        m_customController = new CustomPositionControlLoop(
            Constants.TurretConstants.kGain,
            Constants.TurretConstants.kToleranceDegrees,
            Constants.TurretConstants.kRampUpTime,
            Constants.TurretConstants.kRampDownTime,
            100.0,
            Constants.TurretConstants.kMaxSpeed,
            Constants.TurretConstants.kMinSpeed,
            Constants.TurretConstants.kLoopTime
        );
        
        currentAngle = getTurretAngle();
        setpoint = MathUtil.clamp(currentAngle, Constants.TurretConstants.kMinAngleDegrees, Constants.TurretConstants.kMaxAngleDegrees);
    }

    private double m_previousEncoderValue = 0.0;
    private int m_encoderRevolutions = 0;
    
    public double getTurretAngle() {
        double currentRaw = m_encoder.get();
        
        if (currentRaw < 0.1 && m_previousEncoderValue > 0.9) {
            m_encoderRevolutions++;
        } else if (currentRaw > 0.9 && m_previousEncoderValue < 0.1) {
            m_encoderRevolutions--;
        }
        
        m_previousEncoderValue = currentRaw;
        double totalEncoderRotations = m_encoderRevolutions + currentRaw;
        double angle = (totalEncoderRotations * 360.0) / Constants.TurretConstants.gearRatio;
        
        return angle;
    }

    public void setSetpoint(double degrees) {
        setpoint = MathUtil.clamp(degrees, Constants.TurretConstants.kMinAngleDegrees, Constants.TurretConstants.kMaxAngleDegrees);
    
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

    private double getShortestRotation(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        return error;
    }
    
    private boolean isWithinLimits() {
        return currentAngle >= Constants.TurretConstants.kMinAngleDegrees && currentAngle <= Constants.TurretConstants.kMaxAngleDegrees;
    }
    
    private boolean wouldExceedLimits(double output) {
        if (currentAngle <= Constants.TurretConstants.kMinAngleDegrees + LIMIT_BUFFER && output < 0) {
            return true;
        }
        if (currentAngle >= Constants.TurretConstants.kMaxAngleDegrees - LIMIT_BUFFER && output > 0) {
            return true;
        }
        return false;
    }

    private void setMotorOutput(double output) {
        if (!isWithinLimits()) {
            if (currentAngle < Constants.TurretConstants.kMinAngleDegrees && output > 0) {
                m_motor.set(ControlMode.PercentOutput, output);
            } else if (currentAngle > Constants.TurretConstants.kMaxAngleDegrees && output < 0) {
                m_motor.set(ControlMode.PercentOutput, output);
            } else {
                m_motor.set(ControlMode.PercentOutput, 0.0);
            }
        } else if (wouldExceedLimits(output)) {
            m_motor.set(ControlMode.PercentOutput, 0.0);
        } else {
            m_motor.set(ControlMode.PercentOutput, output);
        }
    }

    @Override
    public void periodic() {
        currentAngle = getTurretAngle();
        
        double smallestError = getShortestRotation(setpoint, currentAngle);
        double motorOutput = m_customController.calculate(smallestError, currentAngle, setpoint);
        
        setMotorOutput(motorOutput / 100.0);
        
        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Setpoint", setpoint);
        SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Motor Output", motorOutput / 100.0);
        SmartDashboard.putNumber("Turret/P Value", m_customController.getP());
        SmartDashboard.putNumber("Turret/Position Error", smallestError);
        SmartDashboard.putNumber("Turret/Raw Angle", m_encoder.get());
        SmartDashboard.putNumber("Turret/Raw Angle With Gear Ratio", m_encoder.get()*Constants.TurretConstants.gearRatio);
        SmartDashboard.putBoolean("Turret/Within Limits", isWithinLimits());
    }
}