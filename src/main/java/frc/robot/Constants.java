// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


    public static class TurretConstants {
      public static final int kTurretMotorID = 5; 
      public static final int kAbsEncoderPort = 0;  
      public static final boolean kMotorInverted = true;  
      public static final double kEncoderOffsetDegrees = 0;  
      
     
      public static final double kMinAngleDegrees = 0;  
      public static final double kMaxAngleDegrees = 360;   
      
     
      public static final double kGain = 1.0;  
      public static final double kMaxSpeed = 40.0;  
      public static final double kMinSpeed = 5.0;  
      
     
      public static final double kToleranceDegrees = 1.0;  
      
      public static final double smallGearToBigGearRatio = 6.7;
     

      public static final double kRampUpTime = 0.6;  
      public static final double kRampDownTime = 0.01;  
      public static final double kLoopTime = 0.02;  
        
    }
}
