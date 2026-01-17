package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

public class Turret extends SubsystemBase {
    private final TalonSRX m_motor;
    private final Encoder m_encoder;
    private final CustomPositionControlLoop m_customController;    
    private double currentAngle;
    private double setpoint;

    public Turret() {
        m_motor = new TalonSRX(Constants.TurretConstants.kTurretMotorID);
        m_motor.setInverted(Constants.TurretConstants.kMotorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);
        
        m_encoder = new Encoder(Constants.TurretConstants.kRelEncoderPort1, Constants.TurretConstants.kRelEncoderPort2);
        m_encoder.setDistancePerPulse(360.0 / (Constants.TurretConstants.kEncoderPulsesPerRotation * Constants.TurretConstants.kGearRatio));
        m_encoder.reset();
        
        m_customController = new CustomPositionControlLoop(
            Constants.TurretConstants.kGain,
            Constants.TurretConstants.kToleranceDegrees,
            Constants.TurretConstants.kRampUpTime,
            Constants.TurretConstants.kRampDownTime,
            Constants.TurretConstants.kUnitsPerRampTime,
            Constants.TurretConstants.kMaxSpeed,
            Constants.TurretConstants.kMinSpeed,
            Constants.TurretConstants.kLoopTime
        );
        
        currentAngle = getTurretAngle();
        setpoint = MathUtil.clamp(currentAngle, Constants.TurretConstants.kMinAngleDegrees, Constants.TurretConstants.kMaxAngleDegrees);
    }
  
    public double getTurretAngle() {  
        double rawAngle = m_encoder.getDistance();
        double wrappedAngle = rawAngle % 360.0;
        if (wrappedAngle < 0) {
            wrappedAngle += 360.0;
        }
        return wrappedAngle;
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

    public void resetEncoder() {
        m_encoder.reset();
    }

    private double getError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        return error;
    }
    
    private boolean isWithinLimits() {
        return currentAngle >= Constants.TurretConstants.kMinAngleDegrees && currentAngle <= Constants.TurretConstants.kMaxAngleDegrees;
    }
    
    private boolean wouldExceedLimits(double output) {
        if (currentAngle <= Constants.TurretConstants.kMinAngleDegrees + Constants.TurretConstants.kTurretLimitBuffer && output < 0) {
            return true;
        }
        if (currentAngle >= Constants.TurretConstants.kMaxAngleDegrees - Constants.TurretConstants.kTurretLimitBuffer && output > 0) {
            return true;
        }
        return false;
    }

    private void setMotorOutput(double output) {
        if(!isWithinLimits()) {
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
        
        double smallestError = getError(setpoint, currentAngle);
        double motorOutput = m_customController.calculate(smallestError, currentAngle, setpoint);
        
        setMotorOutput(motorOutput / 100.0);
        
        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Setpoint", setpoint);
        SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Motor Output", motorOutput/100.0);
        SmartDashboard.putNumber("Turret/P Value", m_customController.getP());
        SmartDashboard.putNumber("Turret/Position Error", smallestError);
        SmartDashboard.putNumber("Turret/Encoder Count", m_encoder.get());
        SmartDashboard.putBoolean("Turret/Within Limits", isWithinLimits());
        SmartDashboard.putNumber("Raw Encoder Count", m_encoder.get());
    }
}