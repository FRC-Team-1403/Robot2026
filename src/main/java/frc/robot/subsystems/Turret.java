package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.CustomPositionControlLoop;

public class Turret extends SubsystemBase {

    private final TalonSRX m_motor;
    private final DutyCycleEncoder m_encoder;
    private final CustomPositionControlLoop m_customController;

    private double currentAngle;
    private double setpoint;

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
        setpoint = currentAngle;
    }

    public double getTurretAngle() {
        double angle = (m_encoder.get() * Constants.TurretConstants.smallGearToBigGearRatio) * 360.0;
        angle -= Constants.TurretConstants.kEncoderOffsetDegrees;
        return MathUtil.inputModulus(angle, 0.0, 360.0);
    }

    public void setSetpoint(double degrees) {
        setpoint = MathUtil.inputModulus(degrees, 0.0, 360.0);
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
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }
        return error;
    }

    private void setMotorOutput(double output) {
        m_motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void periodic() {

        currentAngle = getTurretAngle();

        double smallestError = getShortestRotation(setpoint, currentAngle);
        double goalSetpoint = currentAngle + smallestError;

        double motorOutput = m_customController.calculate(goalSetpoint, currentAngle);

        setMotorOutput(motorOutput / 100.0);

        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Setpoint", setpoint);
        SmartDashboard.putBoolean("Turret/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Motor Output", motorOutput / 100.0);
        SmartDashboard.putNumber("Turret/P Value", m_customController.getP());
        SmartDashboard.putNumber("Turret/Position Error", smallestError);
    }
}