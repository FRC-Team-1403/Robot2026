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

    public static final double kGain = 1.25;
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
    public static double kMinAngleDegrees = Double.NEGATIVE_INFINITY;
    public static double kMaxAngleDegrees = Double.POSITIVE_INFINITY;
  }

  public static class Vision {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static double kGoalY = 4;
    public static double kGoalX = 12;
    public static String kCamera1 = "ThriftyCamera1";
    public static String kCamera2 = "Limelight2";
    public static String kCamera3 = "Limelight3"; 
    public static Transform3d kCamera1Transform = new Transform3d();
    public static Transform3d kCamera2Transform = new Transform3d(0,0,0, new Rotation3d(0,0,Math.toRadians(90)));
    public static Transform3d kCamera3Transform = new Transform3d(0,0,0, new Rotation3d(0, 0, Math.toRadians(180)));
  }
}
