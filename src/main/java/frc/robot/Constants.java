package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TurretConstants {
    public static final int kTurretMotorID = 5;
    public static final int kRelEncoderPort1 = 4;
    public static final int kRelEncoderPort2 = 5;

    public static final boolean kMotorInverted = false;

    public static final double kGain = 0.75;
    public static final double kMaxSpeed = 100.0;
    public static final double kMinSpeed = 7.0;

    public static final double kToleranceDegrees = 1;
    public static final double kGearRatio = 6.7;

    public static final double kRampUpTime = 0.01;
    public static final double kRampDownTime = 0.001;
    public static final double kLoopTime = 0.02;
    public static final double kUnitsPerRampTime = 100;

    public static final double kTurretLimitBuffer = 2.0;

    public static final double kEncoderPulsesPerRotation = 0;
    public static double kMinAngleDegrees = -135;
    public static double kMaxAngleDegrees = 135;
  }

  public static class Vision {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static double kGoalY = 4;
    public static double kGoalX = 12;
    
    public static String kCamera1 = "Limelight";
    public static String kCamera2 = "Thrifty";
    public static String kCamera3 = "OtherThrifty"; 
    public static String kCamera4 = "ThriftyCamera2.0";
    
    public static Transform3d kCamera1Transform = new Transform3d(0.373,-0.294,0.367, new Rotation3d(Math.toRadians(10),0,Math.toRadians(0)));      
    public static Transform3d kCamera2Transform = new Transform3d(-0.377926,-0.31736,0.484686, new Rotation3d(Math.toRadians(10),0,Math.toRadians(90)));     
    public static Transform3d kCamera3Transform = new Transform3d(-0.267,0,0.431, new Rotation3d(Math.toRadians(10),0,Math.toRadians(180)));   
    public static Transform3d kCamera4Transform = new Transform3d(0.33,-0.3,0.431, new Rotation3d(Math.toRadians(10),0,Math.toRadians(270)));   
  }
}