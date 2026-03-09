package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleLib;
import frc.robot.subsystems.CANdleLib.Animations;
import frc.robot.subsystems.CANdleLib.Colors;
import frc.robot.subsystems.CANdleLib.LEDStrip;

public class LEDCommand extends Command{
    private CANdleLib m_CANdleLib;
    private final CANdle m_CANdle;
    private final LEDStrip m_fullStrip;
    private final Animations m_blueStrobe;
    private final Animations m_redStrobe;
    private final Optional<Alliance> m_alliance;
    private boolean m_isRed;

    public LEDCommand(int CANdleId, int ledCount, int globalBrightness) {
        m_CANdleLib = new CANdleLib(0, new LEDConfigs().withStripType(StripTypeValue.RGB), ledCount, globalBrightness);
        m_CANdle = m_CANdleLib.createCANdle();
        m_fullStrip = m_CANdleLib.createLEDStrip(0, ledCount);
        m_blueStrobe = m_CANdleLib.createAnimation(m_CANdle, m_fullStrip, Colors.BLUE, 0.5, 0.2, 0);
        m_redStrobe = m_CANdleLib.createAnimation(m_CANdle, m_fullStrip, Colors.RED, 0.5, 0.2, 0);  
        m_alliance = DriverStation.getAlliance();  
        m_isRed = false;    
    }
    
    @Override
    public void initialize() {
        if (m_alliance.equals(Alliance.Blue)) {
            m_isRed = false;
        }
        else {
            m_isRed = true;
        }
    }

    @Override
    public void execute() {
        if ()
        if (m_isRed) {
            m_redStrobe.run();
        }
        else {
            m_blueStrobe.run();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}