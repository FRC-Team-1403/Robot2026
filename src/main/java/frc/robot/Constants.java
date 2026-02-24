package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class Operator {
    public static final int kOperatorControllerPort = 0;
  }

  public static class Driver {
    public static final int kDriverControllerPort = 0;
  }

  public static class Field {
    // 2026 REBUILT field: 16.54m x 8.07m
    public static final double kFieldLengthMeters = 16.54;
    public static final double kFieldWidthMeters = 8.07;

    public static final Translation2d GOAL_LOCATION = new Translation2d(Vision.kGoalX, Vision.kGoalY);

    // HUB opening front edge height: 72in = 1.83m (game manual §5.4)
    public static final double kGoalHeightMeters = 1.83;
  }

  public static class ShootOnTheFly {
    public static final double kSpindexerFeedRPM = 800.0;
    public static final double kIndexerFeedRPM = 1500.0;

    public static final double kShotToleranceMeters = 0.10;
    public static final double kMaxPoseFOM = 0.5;

    public static final boolean kAutoShootEnabled = true;

    
    public static final double kRedAllianceZoneMinX = 12.51;
    public static final double kBlueAllianceZoneMaxX = 4.03;

    
    public static final Translation2d kRedHubLocation  = new Translation2d(12.51, 4.035);
    public static final Translation2d kBlueHubLocation = new Translation2d(Vision.kGoalX, Vision.kGoalY);

    public static final Translation2d kRedFeedLocationRight  = new Translation2d(15.0, 1.0);
    public static final Translation2d kRedFeedLocationLeft   = new Translation2d(15.0, 7.0);

    public static final Translation2d kBlueFeedLocationRight = new Translation2d(1.5, 1.0);
    public static final Translation2d kBlueFeedLocationLeft  = new Translation2d(1.5, 7.0);
  }

  public static class Turret {
    public static final int kTurretMotorID = 0;
    public static final int kEncoderID = 0;
    public static final int kRelEncoderPort1 = 4;
    public static final int kRelEncoderPort2 = 5;

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

    // Blue HUB center: 4.03m from blue wall, 4.035m from side wall
    public static double kGoalY = 4.035;
    public static double kGoalX = 4.03;

    public static String kCamera1 = "ThriftyCam1.0";
    public static String kCamera2 = "Limelight2";
    public static String kCamera3 = "Limelight3";
    public static String kCamera4 = "ThriftyCamera2.0";
    public static Transform3d kCamera1Transform = new Transform3d();
    public static Transform3d kCamera2Transform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(90)));
    public static Transform3d kCamera3Transform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(180)));
    public static Transform3d kCamera4Transform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(270)));
  }

  public static class ShooterHood {
    public static final int kHoodMotorID = 0;
    public static final int kEncoderID = 0;

    public static final double kMinAngleDegrees = 0;
    public static final double kMaxAngleDegrees = 50;

    public static final double kGain = 0.75;
    public static final double kMaxSpeed = 100.0;
    public static final double kMinSpeed = 7.0;

    public static final double kToleranceDegrees = 1;
    public static final double kGearRatio = 6.7;

    public static final double kRampUpTime = 0.01;
    public static final double kRampDownTime = 0.001;
    public static final double kLoopTime = 0.02;
    public static final double kUnitsPerRampTime = 100;

    public static final double angle1 = 0;
    public static final double angle2 = 0;
    public static final double angle3 = 0;
    public static final double angle4 = 0;
    public static final double angle5 = 0;
    public static final double angle6 = 0;
    public static final double angle7 = 0;
    public static final double angle8 = 0;
  }

  public static class Shooter {
    public static final int flywheelLeaderID = 0;
    public static final int flywheelFollowerID = 0;
    public static final int flywheelFollower2ID = 0;
    public static final double flywheelGearRatio = 0;
    public static final double rpmTolerance = 20;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double rpm1 = 0;
    public static final double rpm2 = 0;
    public static final double rpm3 = 0;
    public static final double rpm4 = 0;
    public static final double rpm5 = 0;
    public static final double rpm6 = 0;
    public static final double rpm7 = 0;
    public static final double rpm8 = 0;
  }

  public static class Indexer {
    public static final int m_indexerID = 0;
    public static final double m_indexerGearRatio = 1.0;
    public static final double rpmTolerance = 20;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double rpm1 = 0;
    public static final double rpm2 = 0;
    public static final double rpm3 = 0;
    public static final double rpm4 = 0;
    public static final double rpm5 = 0;
    public static final double rpm6 = 0;
    public static final double rpm7 = 0;
    public static final double rpm8 = 0;
  }

  public static class Spindexer {
    public static final int m_spindexerID = 0;
    public static final double m_spindexerGearRatio = 1.0;
    public static final double rpmTolerance = 20;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double rpm1 = 0;
    public static final double rpm2 = 0;
    public static final double rpm3 = 0;
    public static final double rpm4 = 0;
    public static final double rpm5 = 0;
    public static final double rpm6 = 0;
    public static final double rpm7 = 0;
    public static final double rpm8 = 0;
  }

  public static class Ranges {
    public static final double dist1 = 0;
    public static final double dist2 = 0;
    public static final double dist3 = 0;
    public static final double dist4 = 0;
    public static final double dist5 = 0;
    public static final double dist6 = 0;
    public static final double dist7 = 0;
    public static final double dist8 = 0;

    public static final double[][] DISTANCE_TO_WHEEL_SPEED = {
        {dist1, Shooter.rpm1},
        {dist2, Shooter.rpm2},
        {dist3, Shooter.rpm3},
        {dist4, Shooter.rpm4},
        {dist5, Shooter.rpm5},
        {dist6, Shooter.rpm6},
        {dist7, Shooter.rpm7},
        {dist8, Shooter.rpm8},
    };

    public static final double[][] DISTANCE_TO_PITCH_DEG = {
        {dist1, ShooterHood.angle1},
        {dist2, ShooterHood.angle2},
        {dist3, ShooterHood.angle3},
        {dist4, ShooterHood.angle4},
        {dist5, ShooterHood.angle5},
        {dist6, ShooterHood.angle6},
        {dist7, ShooterHood.angle7},
        {dist8, ShooterHood.angle8},
    };
  }

  public static class Intake {
    public static final int m_intakeID = 0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.1;
    public static final double kV = 0.1;
    public static final double kA = 0.0;

    public static final double RPM = 0.0;

    public static final int rpmTolerance = 20;

    public static final double intakeMaxVelocity = 5000;
    public static final double intakeMaxAcceleration = 10000;

    public static final double intakeGearRatio = 1.0;

    public static final double intakeCurrentLimit = 40;

    public static final int intakeMotorID = 10;
  }

  public static final class IntakeWrist {
    public static final int kWristMotorID = 2;
    public static final int kEncoderID = 3;

    public static final double kGearRatio = 1.0;
    public static final double kDeployedAngle = 90.0;
    public static final double kStowedAngle = 0.0;
    public static final double kMinAngleDegrees = 0.0;
    public static final double kMaxAngleDegrees = 90.0;

    public static final double kGain = 1.0;
    public static final double kToleranceDegrees = 2.0;
    public static final double kRampUpTime = 0.25;
    public static final double kRampDownTime = 0.25;
    public static final double kUnitsPerRampTime = 10.0;
    public static final double kMaxSpeed = 100.0;
    public static final double kMinSpeed = 5.0;
    public static final double kLoopTime = 0.02;
  }
}