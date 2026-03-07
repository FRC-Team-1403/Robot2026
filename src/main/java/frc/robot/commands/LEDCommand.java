package frc.robot.commands;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleLib;
import frc.robot.subsystems.CANdleLib.Animations;

public class LEDCommand extends Command{
    private CANdleLib m_CANdleLib;
    private final Animations m_blueStrobe;
    private final Animations m_redStrobe;

    public LEDCommand(int CANdleId, int ledCount, int globalBrightness) {
        m_CANdleLib = new CANdleLib(0, new LEDConfigs().withStripType(StripTypeValue.RGB), ledCount, globalBrightness);
        m_blueStrobe = m_CANdleLib.
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}