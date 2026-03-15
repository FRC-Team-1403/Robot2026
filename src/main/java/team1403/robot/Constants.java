package team1403.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
// Variables to used by all subsystems.
  public static final double kLoopTime = 0.02;
  //controls if the debug tab is used on shuffleboard
  public static final boolean DEBUG_MODE = false;
  public static final boolean ENABLE_SYSID = false;
  //controls if the debug tab is used on shuffleboard


  public static class Operator {
    public static final int kOperatorControllerPort = 0;
  }

  public static class Driver {
    public static final int kDriverControllerPort = 0;
  }

  public static class ScoringLocation{
    public static final Translation2d kHubPosition = new Translation2d(8.27, 4.105);
    public static final Translation2d kFeedTopPosition    = new Translation2d(4.03, 6.5);
    public static final Translation2d kFeedBottomPosition = new Translation2d(4.03, 1.5);
  }

  public static class Turret {
    public static final int kTurretMotorID = 21;
    public static final int kEncoderID = 22;
    public static final double kMagnetOffset = 0.19775390625;

    public static final double kMinAngleDegrees = -180.0;
    public static final double kMaxAngleDegrees = 180.0;

    public static final double kGain = 0.6;
    public static final double kMaxSpeed = 35.0;
    public static final double kMinSpeed = 2.0;

    public static final double kGearRatioEncoder = (85.0 / 10.0); // 12.0 50
    public static final double kGearRatioTurretAngleRatio = (50.0 / 12.0) * (85.0 / 10.0);

    public static final double kToleranceDegrees = 0.25;
    public static final double kGearRatio = (56.0 / 16.0);

    public static final double kRampUpTime = 0.01;
    public static final double kRampDownTime = 0.001;
    public static final double kLoopTime = 0.02;
    public static final double kUnitsPerRampTime = 100;

    public static final Translation2d kTurretOffset = new Translation2d(0.1, 0.05); //tune
  }

  public static class Swerve {
    public static final double KpDrive = 0.14923;
    public static final double KiDrive = 0;
    public static final double KdDrive = 0;

    public static final double KsDrive = 0.13382;
    public static final double KvDrive = 0.11367;
    public static final double KaDrive = 0.0074265;

    public static final double KpSteer = 65;
    public static final double KiSteer = 0;
    public static final double KdSteer = 0;

    public static final double KsSteer = 0.01;
    public static final double KvSteer = 2.62;
    public static final double KaSteer = 0;
  }
  
  public static class Vision {
    public static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static double kGoalY = 4;
    public static double kGoalX = 12;
    public static String kCamera1 = "ThriftyCam1.0";
    public static String kCamera2 = "ThriftyCam2.0";
    public static String kCamera3 = "ThriftyCam3.0";
    public static String kCamera4 = "ThriftyCam4.0";
    public static Transform3d kCamera1Transform = new Transform3d();
    public static Transform3d kCamera2Transform =
        new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(90)));
    public static Transform3d kCamera3Transform =
        new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(180)));
    public static Transform3d kCamera4Transform =
        new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(270)));
  
    //ETAASH METHOD + MATCHED TO THRIFTY. FIX THESE:
    public static final Translation3d kCameraOffset = new Translation3d();

    public static final boolean kExtraVisionDebugInfo = true;

    public static final Transform3d kCameraTransfromThriftyCamera1 = new Transform3d(kCameraOffset, new Rotation3d(0,0,0));;
    public static final Transform3d kCameraTransfromThriftyCamera2 = new Transform3d(kCameraOffset, new Rotation3d(0,0, Math.toRadians(90)));
    public static final Transform3d kCameraTransfromThriftyCamera3 = new Transform3d(kCameraOffset, new Rotation3d(0,0, Math.toRadians(180)));
    public static final Transform3d kCameraTransfromThriftyCamera4 = new Transform3d(kCameraOffset, new Rotation3d(0,0, Math.toRadians(270)));


    //NEW COMBINED VISION SUBSYSTEM
  }

  public static class ShooterHood {
    public static final int kHoodMotorID = 26;
    public static final int kEncoderID = 27;

    public static final double kMinAngleDegrees = 0.1;
    public static final double kMaxAngleDegrees = 30;

    public static final double kGain = 4;
    public static final double kMaxSpeed = 12.0;
    public static final double kMinSpeed = 2.0;

    public static final double kToleranceDegrees = 0.3;
    public static final double kGearRatioEncoder = (56.0 / 16.0);
    public static final double kGearRatioHoodAngleRatio = (56.0 / 16.0) * (175.0 / 10.0);

    public static final double kRampUpTime = 0.01;
    public static final double kRampDownTime = 0.001;
    public static final double kLoopTime = 0.02;
    public static final double kUnitsPerRampTime = 100;

    public static final double kMagnetOffset = -0.89111328125;

    public static final double kS = 0.0;
    public static final double kG = 2.5;
    public static final double kV = 0;
    public static final double kA = 0;
  }

  public static class Shooter {
    public static final int flywheelLeaderID = 23;
    public static final int flywheelFollower1TopRightID = 24;
    public static final int flywheelFollower2BottonRightID = 25;
    public static final double flywheelGearRatio = 1;
    public static final double rpmTolerance = 90.0;

    public static final double kP = 0.075;
    public static final double kI = 0;
    public static final double kD = 0.001;
    public static final double kS = 0.1;
    public static final double kV = 0.13;
    public static final double kA = 0.2;
  }

  public static class Indexer {
    public static final int m_indexerID = 32;
    public static final double m_indexerGearRatio = 1.0;
    public static final double rpmTolerance = 20;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
  }

  public static class Spindexer {
    public static final int m_spindexerID = 31;
    public static final double m_spindexerGearRatio = 1.0;
    public static final double rpmTolerance = 20;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
  }

  public static class Intake {
    public static final int m_intakeTurretSideLeaderID = 41;
    public static final int m_intakeNonTurretSideFollowerID = 42;

    public static final double rollerRPM = 5000;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.1;
    public static final double kV = 0.1;
    public static final double kA = 0.0;

    public static final int rpmTolerance = 20;
    public static final double intakeGearRatio = 1.0;
    public static final int intakeMotorID = 10;
  }

  public static final class IntakeWrist {
    public static final int kWristMotorID = 43;
    public static final int kEncoderID = 44;

    public static final double kMagnetOffset = 1;
    public static final double kGearRatioEncoder = 1;
    public static final double kGearRatioWristAngleRatio = 1;

    public static final double kGearRatio = 1.0;
    public static final double kDeployedAngle = 90.0;
    public static final double kStowedAngle = 0.0;
    public static final double kMinAngleDegrees = 0.0;
    public static final double kMaxAngleDegrees = 90.0;

    public static final double  wristRPMStartAngle = 60.0;

    public static final double kGain = 1.0;
    public static final double kToleranceDegrees = 2.0;
    public static final double kRampUpTime = 0.25;
    public static final double kRampDownTime = 0.25;
    public static final double kUnitsPerRampTime = 10.0;
    public static final double kMaxSpeed = 100.0;
    public static final double kMinSpeed = 5.0;
    public static final double kLoopTime = 0.02;
  }

  public static final class Hopper {
    public static final int kCANRangeID = 51; 

    public static final double kMaxDistanceMeters = 0.50;
    public static final double kMinDistanceMeters = 0.05; 
    public static final double kFullThresholdPercentage = 90.0;
    public static final double kEmptyThresholdPercentage = 5.0;
  }

  public static final class ShooterTuning {

    public static final class myAllianceZone {

      public static final double range1Distance = 1.0;
      public static final double range2Distance = 1.5;
      public static final double range3Distance = 2.0;
      public static final double range4Distance = 2.5;
      public static final double range5Distance = 3.0;
      public static final double range6Distance = 3.5;
      public static final double range7Distance = 4.0;
      public static final double range8Distance = 4.5;

      public static final double range1IndexerRpm = 1500;
      public static final double range1SpindexerRpm = 1000;

      public static final double range1ShooterRpm = 1800;
      public static final double range1HoodAngle = 10;

      public static final double range2ShooterRpm = 1900;
      public static final double range2HoodAngle = 12;

      public static final double range3ShooterRpm = 2000;
      public static final double range3HoodAngle = 14;

      public static final double range4ShooterRpm = 2100;
      public static final double range4HoodAngle = 16;

      public static final double range5ShooterRpm = 2200;
      public static final double range5HoodAngle = 18;

      public static final double range6ShooterRpm = 2300;
      public static final double range6HoodAngle = 20;

      public static final double range7ShooterRpm = 2400;
      public static final double range7HoodAngle = 22;

      public static final double range8ShooterRpm = 2500;
      public static final double range8HoodAngle = 24;
    }

    public static final class neutralZone {

      public static final double range1Distance = 1.2;
      public static final double range2Distance = 1.7;
      public static final double range3Distance = 2.2;
      public static final double range4Distance = 2.7;
      public static final double range5Distance = 3.2;
      public static final double range6Distance = 3.7;
      public static final double range7Distance = 4.2;
      public static final double range8Distance = 4.7;

      public static final double range1IndexerRpm = 1600;
      public static final double range1SpindexerRpm = 1100;

      public static final double range1ShooterRpm = 1900;
      public static final double range1HoodAngle = 12;

      public static final double range2ShooterRpm = 2000;
      public static final double range2HoodAngle = 14;

      public static final double range3ShooterRpm = 2100;
      public static final double range3HoodAngle = 16;

      public static final double range4ShooterRpm = 2200;
      public static final double range4HoodAngle = 18;

      public static final double range5ShooterRpm = 2300;
      public static final double range5HoodAngle = 20;

      public static final double range6ShooterRpm = 2400;
      public static final double range6HoodAngle = 22;

      public static final double range7ShooterRpm = 2500;
      public static final double range7HoodAngle = 24;

      public static final double range8ShooterRpm = 2600;
      public static final double range8HoodAngle = 26;
    }

    public static final class opposingAllianceZone {

      public static final double range1Distance = 1.5;
      public static final double range2Distance = 2.0;
      public static final double range3Distance = 2.5;
      public static final double range4Distance = 3.0;
      public static final double range5Distance = 3.5;
      public static final double range6Distance = 4.0;
      public static final double range7Distance = 4.5;
      public static final double range8Distance = 5.0;

      public static final double range1IndexerRpm = 1700;
      public static final double range1SpindexerRpm = 1200;

      public static final double range1ShooterRpm = 2000;
      public static final double range1HoodAngle = 14;

      public static final double range2ShooterRpm = 2100;
      public static final double range2HoodAngle = 16;

      public static final double range3ShooterRpm = 2200;
      public static final double range3HoodAngle = 18;

      public static final double range4ShooterRpm = 2300;
      public static final double range4HoodAngle = 20;

      public static final double range5ShooterRpm = 2400;
      public static final double range5HoodAngle = 22;

      public static final double range6ShooterRpm = 2500;
      public static final double range6HoodAngle = 24;

      public static final double range7ShooterRpm = 2600;
      public static final double range7HoodAngle = 26;

      public static final double range8ShooterRpm = 2700;
      public static final double range8HoodAngle = 28;
    }
  }
}
