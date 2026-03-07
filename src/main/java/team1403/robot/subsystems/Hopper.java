package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private final CANrange m_canRange;

    public Hopper() {
        m_canRange = new CANrange(Constants.Hopper.kCANRangeID);

        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
        m_canRange.getConfigurator().apply(canRangeConfig);
    }

    public double getDistanceMeters() {
        return m_canRange.getDistance().getValueAsDouble();
    }

    public double getFillPercentage() {
        double distance = getDistanceMeters();
        double clamped = Math.max(Constants.Hopper.kMinDistanceMeters, Math.min(Constants.Hopper.kMaxDistanceMeters, distance));
        double percentage = (1.0 - (clamped - Constants.Hopper.kMinDistanceMeters) / (Constants.Hopper.kMaxDistanceMeters - Constants.Hopper.kMinDistanceMeters)) * 100.0;
        return percentage;
    }

    public boolean isFull() {
        return getFillPercentage() >= Constants.Hopper.kFullThresholdPercentage;
    }

    public boolean isEmpty() {
        return getFillPercentage() <= Constants.Hopper.kEmptyThresholdPercentage;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Hopper/Fill Percentage", getFillPercentage());
        Logger.recordOutput("Hopper/Distance Meters", getDistanceMeters());
        Logger.recordOutput("Hopper/Is Full", isFull());
        Logger.recordOutput("Hopper/Is Empty", isEmpty());
        Logger.recordOutput("Hopper/Signal Strength", m_canRange.getSignalStrength().getValueAsDouble());
        Logger.recordOutput("Hopper/Measurement Valid", m_canRange.getIsDetected().getValue());
    }
}