package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private TalonFX m_motor;

    public Hopper () {
        m_motor = new TalonFX(0);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.CurrentLimits.StatorCurrentLimit = 40;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = false; // false or true

        m_motor.getConfigurator().apply(motorConfig);
    }

    public void setMotorPower(double speed) {
        m_motor.set(speed);
    }

    public double getMotorPower(){
        return m_motor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("HopperSpeed", getMotorPower());
    }
    
}
