package frc.robot;

public final class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    public static class TurretConstants {
      public static final int kTurretMotorID = 5; 
      public static final int kHallEffectPort = 9; 
      public static final int kAbsEncoderPort = 0;  
      public static final boolean kMotorInverted = true;  
     
      public static final double kMinAngleDegrees = 0;  
      public static final double kMaxAngleDegrees = 360;   
        
      public static final double kGain = 2.0;  
      public static final double kMaxSpeed = 40.0;  
      public static final double kMinSpeed = 5.0;  
        
      public static final double kToleranceDegrees = 1.0;    
      public static final double gearRatio = 6.7;
     
      public static final double kRampUpTime = 0.6;  
      public static final double kRampDownTime = 0.01;  
      public static final double kLoopTime = 0.02;      
    }
}
