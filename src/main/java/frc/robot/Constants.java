package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;


public final class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    public static class TurretConstants {
      public static final int kTurretMotorID = 5; 
      public static final int kAbsEncoderPort = 0;  
      public static final boolean kMotorInverted = true;  
     
      public static final double kMinAngleDegrees = 0;  
      public static final double kMaxAngleDegrees = 270;   
        
      public static final double kGain = 2.0;  
      public static final double kMaxSpeed = 40.0;  
      public static final double kMinSpeed = 5.0;  
        
      public static final double kToleranceDegrees = 1.0;    
      public static final double kGearRatio = 6.7;
     
      public static final double kRampUpTime = 0.5;  
      public static final double kRampDownTime = 0.001;  
      public static final double kLoopTime = 0.02;   
      public static final double kUnitsPerRampTime = 100;
      
      public static final double kTurretLimitBuffer = 2.0;
    }

    public static class Vision {
    public static final String kCameraName = "";
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static Transform3d kRobotToCamera;
    public static double kGoalY = 0;
    public static double kGoalX = 0;
    }
}
