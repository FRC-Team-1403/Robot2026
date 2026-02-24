package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import frc.robot.CANdle.CANdleLib;
import frc.robot.CANdle.CANdleLib.LEDStrip;
import frc.robot.CANdle.CANdleLib.Animations;



import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;



public class LED_Timer extends Command {
    Animations shift1;
    Animations shift2;
    Animations endgame;
    CANdleLib m_CANdleLib;
    CANdle m_candle;
    LEDStrip fullstrip;
    Timer m_timer;
    int led_Pixels = 10;



// Red Alliance
    public LED_Timer(CANdleLib candle, Timer m_timer, LEDStrip fullstrip,  Animations shift1, Animations shift2, Animations endgame ){
        m_CANdleLib = new CANdleLib(Constants.candle.id, new LEDConfigs().withStripType(StripTypeValue.RGB), 10, 0.1);

        this.m_CANdleLib = candle;
       this.fullstrip = m_CANdleLib.createLEDStrip(0, led_Pixels - 1);
       this.m_timer = m_timer;

       this.shift1 = shift1;
       this.shift2 = shift2;
       this.endgame = endgame;
    }
        

    public void initialize(){   
        m_timer.reset();
        m_timer.start();
        endgame.run();
    }

    public void execute() {
        // Time Values(Time remaining frmo the game):  
        //2:20-2:10.   Both Red and Blue
        //2:10-1:45.   Red
        //1:45-1:20.   Blue
        //1:20-0:55.   Red
        //0:55-0:30.   Blue
        //0:30-0:00.   Endgame (Both Red and Blue)

        int phase = (int) (Math.floor((m_timer.get() - 30) / 25));
        if (phase < 0) phase = 0;
        

        if (phase % 2 == 1 && phase > 0){
            shift1.run();
            shift2.stop();
            endgame.stop();    
        } else {
            shift1.stop();
            shift2.run();
            endgame.stop(); 
        }
        
        if (phase >= 4 && m_timer.get() >= 110 ){
            shift1.stop();
            shift2.stop();
            endgame.run();
        }
    }

        
    public void end(boolean interrupted){
        if (m_timer.get() >= 140.0){
            shift2.stop(); 
            endgame.stop();
            shift1.stop();
        }
   
    }
}


    




