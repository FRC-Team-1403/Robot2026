package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LED_Timer;
import frc.robot.CANdle.CANdleLib;
import frc.robot.CANdle.CANdleLib.Animations;
import frc.robot.CANdle.CANdleLib.Colors;
import frc.robot.CANdle.CANdleLib.LEDStrip;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;


public class RobotContainer {
  private final int led_count = 10; 
  private final CANdleLib candleLib = new CANdleLib(Constants.candle.id, new LEDConfigs().withStripType(StripTypeValue.RGB), led_count, 0.1);
  private final CANdle candle = candleLib.createCANdle();
  
  private final LEDStrip topStrip = candleLib.createLEDStrip(0, 3);
  private final LEDStrip middleStrip = candleLib.createLEDStrip(3, 5);
  private final LEDStrip bottomStrip = candleLib.createLEDStrip(5, 8);
  private final LEDStrip fullStrip = candleLib.createLEDStrip(0, 8);

  private final LEDStrip half_1Strip = candleLib.createLEDStrip(0, led_count / 2);
  private final LEDStrip half_2Strip = candleLib.createLEDStrip((led_count / 2) + 1, led_count - 1);

  private Timer m_timer = new Timer();
 


  private final Animations breatheBlue = candleLib.createAnimation(
      candle, fullStrip, Colors.BLUE, 0.5, 0.2, 0
  );
  
  private final Animations breatheRed = candleLib.createAnimation(
      candle, fullStrip, Colors.RED, 1.5, 0.15, 0
  );

  private final Animations breatheGreen = candleLib.createAnimation(
      candle, fullStrip, Colors.GREEN, 1.0, 0.25, 0
  );

  private final Animations breathePurple = candleLib.createAnimation(
      candle, fullStrip, Colors.PURPLE, 0.8, 0.3, 0
  );

  private final Animations countdown15sec = candleLib.createAnimation(
      candle, fullStrip, 15.0, Colors.ORANGE
  );

  private final Animations anim1 =  candleLib.createAnimation(
      candle, topStrip, Colors.RED, 0.2, 0.2, 0
  );

  private final Animations anim2 = candleLib.createAnimation(
      candle, middleStrip, Colors.YELLOW, 0.2, 0.2, (2*Math.PI/3)
  );

  private final Animations anim3 = candleLib.createAnimation(
      candle, bottomStrip, Colors.BLUE, 0.2, 0.2, (4*Math.PI/3)
  );

  private final Animations strobeAnim = candleLib.createAnimation(
      candle, fullStrip, Colors.WHITE, 5.0, 0.0, 0
  );

    private Supplier<Boolean> on_off(Timer timer) {
        timer.restart(); 
        return () -> (Math.floor(timer.get()) % 2 == 1);
}

  Supplier<Boolean> bool = on_off(m_timer);
  private final Animations red_shiftAnimations = candleLib.createAnimation(candle, fullStrip, 10, CANdleLib.Colors.RED);
  private final Animations blue_shiftAnimations = candleLib.createAnimation(candle, fullStrip, 10, CANdleLib.Colors.BLUE);
  private final Animations endgame_Animations = candleLib.createAnimation(candle, fullStrip, bool, CANdleLib.Colors.RED, CANdleLib.Colors.BLUE);
    // createAnimation(CANdle candle, LEDStrip strip, Supplier<Boolean> state, LEDColor trueColor, LEDColor falseColor)



  private enum RobotState {
    IDLE, INTAKING, SHOOTING, CLIMBING
  }
  
  private RobotState currentState = RobotState.IDLE;
  
  private final Animations stateIndicator = candleLib.createAnimation(
      candle, 
      fullStrip, 
      () -> currentState,
      Colors.WHITE,
      Colors.GREEN,
      Colors.RED,
      Colors.PURPLE
  );

  private boolean systemEnabled = false;
  
  private final Animations boolIndicator = candleLib.createAnimation(
      candle,
      fullStrip,
      () -> systemEnabled,
      Colors.GREEN,
      Colors.RED
  );

  private double powerLevel = 50.0;
  
  private final Animations powerBar = candleLib.createAnimation(
      candle,
      fullStrip,
      0.0,
      100.0,
      () -> powerLevel,
      Colors.CYAN,
      Colors.OFF
  );

  private final CommandXboxController driver = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    
    driver.button(1).onTrue(new InstantCommand(() -> {
        stopAll();
        breatheBlue.run();
    }));
    
    driver.button(2).onTrue(new InstantCommand(() -> {
        stopAll();
        breatheRed.run();
    }));
    
    driver.button(3).onTrue(new InstantCommand(() -> {
        stopAll();
        breatheGreen.run();
    }));
    
    driver.button(4).onTrue(new InstantCommand(() -> {
        stopAll();
        breathePurple.run();
    }));

    driver.button(5).onTrue(new InstantCommand(() -> {
        stopAll();
        candleLib.setStripColor(candle, topStrip, Colors.RED).schedule();
        candleLib.setStripColor(candle, middleStrip, Colors.GREEN).schedule();
        candleLib.setStripColor(candle, bottomStrip, Colors.BLUE).schedule();
    }));

    driver.button(6).onTrue(new InstantCommand(() -> {
        stopAll();
        countdown15sec.run();
    }));

    driver.button(7).onTrue(new InstantCommand(() -> {
        stopAll();
        stateIndicator.run();
        currentState = RobotState.values()[
            (currentState.ordinal() + 1) % RobotState.values().length
        ];
    }));

    driver.button(8).onTrue(new InstantCommand(() -> {
        stopAll();
        systemEnabled = !systemEnabled;
        boolIndicator.run();
    }));

    driver.button(9).onTrue(new InstantCommand(() -> {
        stopAll();
        powerBar.run();
    }));


    // driver.button(10).onTrue(getAutonomousCommand())

    driver.button(11).onTrue(new InstantCommand(() -> stopAll()));

    driver.povUp().onTrue(new InstantCommand(() -> {
        powerLevel = Math.min(100.0, powerLevel + 10.0);
        powerBar.run();
    }));

    driver.povDown().onTrue(new InstantCommand(() -> {
        powerLevel = Math.max(0.0, powerLevel - 10.0);
        powerBar.run();
    }));

    driver.povLeft().onTrue(new InstantCommand(() -> {
        stopAll();
        anim1.run();
        anim2.run();
        anim3.run();
    })
  );

    driver.povRight().onTrue(new InstantCommand(() -> {
        stopAll();
        strobeAnim.run();
    }));

    new InstantCommand(() -> new LED_Timer(candleLib, m_timer, fullStrip, red_shiftAnimations, blue_shiftAnimations, endgame_Animations));
  }

  private void stopAll() {
    breatheBlue.end();
    breatheRed.end();
    breatheGreen.end();
    breathePurple.end();
    countdown15sec.end();
    stateIndicator.end();
    boolIndicator.end();
    powerBar.end();
    anim1.end();
    anim2.end();
    anim3.end();
    strobeAnim.end();
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(() -> countdown15sec.run());
  }
}