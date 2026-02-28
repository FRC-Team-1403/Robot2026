package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggingChecker extends SubsystemBase {

    private final String[] keys = {"ShooterRPM", "ArmAngle", "IntakeSpeed"};

    private final NetworkTable table;

    public LoggingChecker() {
        table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    }

    @Override
    public void periodic() {
        for (int i = 0; i < keys.length; i++) {
            NetworkTableEntry entry = table.getEntry(keys[i]);

            if (!entry.exists()) {
                System.out.println(keys[i] + " does not exist/not returning a value.");
            }
        }
    }
}
