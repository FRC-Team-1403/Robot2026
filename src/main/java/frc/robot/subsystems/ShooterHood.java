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

public class ShooterHood extends SubsystemBase {
    private final TalonSRX m_hoodMotor;

    private final CustomPositionControlLoop m_customController;

    private double currentAngle;
    private double setpointAngle;

    public ShooterHood() {
        m_hoodMotor = new TalonSRX(Constants.Shooter.kHoodMotorID);

        m_hoodMotor.setNeutralMode(NeutralMode.Brake);

        m_customController = new CustomPositionControlLoop(
                Constants.TurretConstants.kGain,
                Constants.TurretConstants.kToleranceDegrees,
                Constants.TurretConstants.kRampUpTime,
                Constants.TurretConstants.kRampDownTime,
                Constants.TurretConstants.kUnitsPerRampTime,
                Constants.TurretConstants.kMaxSpeed,
                Constants.TurretConstants.kMinSpeed,
                Constants.TurretConstants.kLoopTime);

    }

    public void setRPM(double rpm) {
        // implement later ish
    }

    @Override
    public void periodic() {
        
    }
}