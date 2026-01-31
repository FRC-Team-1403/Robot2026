package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CustomPositionControlLoop;

public class Shooter extends SubsystemBase {
    private final TalonSRX m_leftMotor;
    private final TalonSRX m_rightMotor;
    private final CustomPositionControlLoop m_customController;

    private double currentRPM;
    private double setpointRPM;

    public Shooter() {
        m_leftMotor = new TalonSRX(Constants.Shooter.kLeftMotorID);
        m_rightMotor = new TalonSRX(Constants.Shooter.kRightMotorID);

        m_rightMotor.setInverted(true);
        m_rightMotor.follow(m_leftMotor);

        m_leftMotor.setNeutralMode(NeutralMode.Brake);
        m_rightMotor.setNeutralMode(NeutralMode.Brake);

        m_customController = new CustomPositionControlLoop(
                Constants.TurretConstants.kGain,
                Constants.TurretConstants.kToleranceDegrees,
                Constants.TurretConstants.kRampUpTime,
                Constants.TurretConstants.kRampDownTime,
                Constants.TurretConstants.kUnitsPerRampTime,
                Constants.TurretConstants.kMaxSpeed,
                Constants.TurretConstants.kMinSpeed,
                Constants.TurretConstants.kLoopTime);

        currentRPM = 0;
        setpointRPM = currentRPM;
    }

    public void setRPM(double rpm) {
        // implement later ish
    }

    @Override
    public void periodic() {
        
    }
}