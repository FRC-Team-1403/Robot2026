package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase{
    private TalonFX m_hooperUp;
    private TalonFX m_shooterSpin;

    public SpindexerSubsystem() {
        m_hooperUp = new TalonFX(1, "Bus 2"); //Fix
        m_shooterSpin = new TalonFX(0, "Bus 2");
    }

    public void setHopperSpeed(double speed) {
        m_hooperUp.set(speed);
    }


    public void setShooterSpeed(double speed) {
        m_shooterSpin.set(speed);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Hopper Temp", m_hooperUp.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Shooter Temp", m_shooterSpin.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Hopper Speed", m_hooperUp.get());
        Logger.recordOutput("Shooter Speed", m_shooterSpin.get());
    }
}
