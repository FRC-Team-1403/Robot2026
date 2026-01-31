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

    // public static final double kMinAngleDegrees = 0;
    // public static final double kMaxAngleDegrees = 360;

    public static final double kGain = 0.75; //1.25
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
    public static String kCamera1 = "ThriftyCam1.0";
    public static String kCamera2 = "Limelight2";
    public static String kCamera3 = "Limelight3"; 
    public static String kCamera4 = "ThriftyCamera2.0";
    public static Transform3d kCamera1Transform = new Transform3d();
    public static Transform3d kCamera2Transform = new Transform3d(0,0,0, new Rotation3d(0,0,Math.toRadians(90)));
    public static Transform3d kCamera3Transform = new Transform3d(0,0,0, new Rotation3d(0, 0, Math.toRadians(180)));
    public static Transform3d kCamera4Transform = new Transform3d(0,0,0,new Rotation3d(0,0,Math.toRadians(270)));
  }

  public static class ShooterHood {
    public static final int kHoodMotorID = 0; // temp

    public static final double kMinAngleDegrees = 0; // temp
    public static final double kMaxAngleDegrees = 50; // temp

    public static final double kGain = 0.75; //1.25
    public static final double kMaxSpeed = 100.0;
    public static final double kMinSpeed = 7.0;

    public static final double kToleranceDegrees = 1;
    public static final double kGearRatio = 6.7;

    public static final double kRampUpTime = 0.01;
    public static final double kRampDownTime = 0.001;
    public static final double kLoopTime = 0.02;
    public static final double kUnitsPerRampTime = 100;

    public static final double superCloseAngle = 0;
    public static final double kindaCloseAngle = 1;
    public static final double closeAngle = 2;
    public static final double kindaMediumAngle = 3;
    public static final double mediumAngle = 4;
    public static final double kindaFarAngle = 5;
    public static final double farAngle = 6;
    public static final double veryFarAngle = 7; 
  }

  public static class Shooter {
    public static final int kLeftMotorID = 0; // temp
    public static final int kRightMotorID = 0; // temp

    public static final double rpmTolerance = 20;

    public static final double kGain = 0.75; //1.25
    public static final double kMaxSpeed = 100.0;
    public static final double kMinSpeed = 7.0;

    public static final double kToleranceDegrees = 1;
    public static final double kGearRatio = 6.7;

    public static final double kRampUpTime = 0.01;
    public static final double kRampDownTime = 0.001;
    public static final double kLoopTime = 0.02;
    public static final double kUnitsPerRampTime = 100;

    public static final double superCloseDist = 5; // everything is temp
    public static final double kindaCloseDist = 10;
    public static final double closeDist = 15;
    public static final double kindaMediumDist = 20;
    public static final double mediumDist = 25;
    public static final double kindaFarDist = 30;
    public static final double farDist = 35;
    public static final double veryFarDist = 40;

    public static final double superCloseRPM = 500;
    public static final double kindaCloseRPM = 1000;
    public static final double closeRPM = 1500;
    public static final double kindaMediumRPM = 2000;
    public static final double mediumRPM = 2500;
    public static final double kindaFarRPM = 3000;
    public static final double farRPM = 3500;
    public static final double veryFarRPM = 4000; 
  }
}
