package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Turret extends SubsystemBase {
    private final TalonSRX m_motor;
    private final DutyCycleEncoder m_encoder;

    private double currentAngle;
    private double currMotorOutput;
    private double desiredMotorOutput;
    private boolean isRampDone;
    private boolean isGoingClockWise;
    private boolean isGoingCounterClockWise;
    private boolean directionFlag;
    private double posError;
    private double setpoint;
    private double minSpeed;
    private double maxSpeed;

    public Turret() {
        m_motor = new TalonSRX(Constants.TurretConstants.kTurretMotorID);
        m_motor.setInverted(Constants.TurretConstants.kMotorInverted);
        m_motor.setNeutralMode(NeutralMode.Brake);

        m_encoder = new DutyCycleEncoder(Constants.TurretConstants.kAbsEncoderPort);

        currentAngle = getTurretAngle();
        setpoint = currentAngle;
        currMotorOutput = 0.0;
        desiredMotorOutput = 0.0;
        isRampDone = false;
        isGoingClockWise = false;
        isGoingCounterClockWise = false;
        directionFlag = true;
    }

    public double getTurretAngle() {
        double angle =(m_encoder.get() * Constants.TurretConstants.smallGearToBigGearRatio)*360;
        angle -= Constants.TurretConstants.kEncoderOffsetDegrees;
        return MathUtil.inputModulus(angle, 0.0, 360.0);
    }


    public void setSetpoint(double degrees) {
        degrees = MathUtil.inputModulus(degrees, 0.0, 360.0);
        setpoint = degrees;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(getShortestRotation(setpoint, currentAngle)) <= Constants.TurretConstants.kToleranceDegrees;
    }

    public void adjustSetpoint(double degrees) {
        setSetpoint(setpoint + degrees);
    }

    public void stopMotor() {
        m_motor.set(ControlMode.PercentOutput, 0.0);
        currMotorOutput = 0.0;
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

    private void checkDirection(double targetSetpoint) {
        double shortestRotation = getShortestRotation(targetSetpoint, currentAngle);
        
        if (Math.abs(shortestRotation) <= Constants.TurretConstants.kToleranceDegrees) {
            isGoingClockWise = false;
            isGoingCounterClockWise = false;
        } else if (shortestRotation > 0) {
            isGoingClockWise = true;
            isGoingCounterClockWise = false;
        } else {
            isGoingClockWise = false;
            isGoingCounterClockWise = true;
        }
    }

    private double getDesiredOutput(double targetSetpoint) {
        posError = getShortestRotation(targetSetpoint, currentAngle);

        if (isGoingClockWise || isGoingCounterClockWise) {
            minSpeed = Constants.TurretConstants.kMinSpeed;
            maxSpeed = Constants.TurretConstants.kMaxSpeed;
        }

        double desiredOutput = Math.abs(posError) * Constants.TurretConstants.kGain;

        if ((!isGoingClockWise && !isGoingCounterClockWise)) {
            desiredOutput = 0;
        }

        if (desiredOutput > maxSpeed) {
            desiredOutput = maxSpeed;
        }

        return desiredOutput;
    }

    private double ramp(double rampUpTime, double rampDownTime, double currentOutput, double desiredOutput) {
        double rampUpRate = 100.0 / (rampUpTime / Constants.TurretConstants.kLoopTime);
        double rampDownRate = 100.0 / (rampDownTime / Constants.TurretConstants.kLoopTime);

        if (desiredOutput > currentOutput) {
            if ((currentOutput + rampUpRate) < desiredOutput) {
                currentOutput += rampUpRate;
                isRampDone = false;
            } else {
                currentOutput = desiredOutput;
                isRampDone = true;
            }
        } else if (desiredOutput < currentOutput) {
            if ((currentOutput - rampDownRate) > desiredOutput) {
                currentOutput -= rampDownRate;
                isRampDone = false;
            } else {
                currentOutput = desiredOutput;
                isRampDone = true;
            }
        } else {
            isRampDone = true;
        }

        return currentOutput;
    }

    private void adjustCurrentOutput() {
        if ((isGoingClockWise || isGoingCounterClockWise) && isRampDone 
            && currMotorOutput < minSpeed) {
            currMotorOutput = minSpeed;
        }

        if (isGoingCounterClockWise) {
            currMotorOutput *= -1;
        }
    }

    private void checkIfReachedSetPoint(double targetSetpoint) {
        if (Math.abs(getShortestRotation(targetSetpoint, currentAngle)) <= Constants.TurretConstants.kToleranceDegrees) {
            currMotorOutput = 0;
            isGoingClockWise = false;
            isGoingCounterClockWise = false;
            directionFlag = true;
        }
    }

    private void setMotorOutput(double output) {
        m_motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void periodic() {
        currentAngle = getTurretAngle();

        if (directionFlag && Math.abs(getShortestRotation(setpoint, currentAngle)) > Constants.TurretConstants.kToleranceDegrees) {
            checkDirection(setpoint);
            directionFlag = false;
        }

        desiredMotorOutput = getDesiredOutput(setpoint);

        if (isGoingClockWise || isGoingCounterClockWise) {
            currMotorOutput = ramp(
                Constants.TurretConstants.kRampUpTime,
                Constants.TurretConstants.kRampDownTime,
                Math.abs(currMotorOutput),
                desiredMotorOutput
            );
        }

        adjustCurrentOutput();
        checkIfReachedSetPoint(setpoint);
        setMotorOutput(currMotorOutput / 100.0);

        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Setpoint", setpoint);
        SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Motor Output", currMotorOutput / 100.0);
        SmartDashboard.putNumber("Turret/Desired Output", desiredMotorOutput);
        SmartDashboard.putNumber("Turret/Position Error", posError);
        SmartDashboard.putBoolean("Turret/Is Going CW", isGoingClockWise);
        SmartDashboard.putBoolean("Turret/Is Going CCW", isGoingCounterClockWise);
        SmartDashboard.putBoolean("Turret/Ramp Done", isRampDone);
    }
}