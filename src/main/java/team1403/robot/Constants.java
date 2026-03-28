package team1403.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
// Variables to used by all subsystems.
  public static final double kLoopTime = 0.02;
  //controls if the debug tab is used on shuffleboard
  public static final boolean DEBUG_MODE = false;
  public static final boolean ENABLE_SYSID = true;
  //controls if the debug tab is used on shuffleboard


  public static class Operator {
    public static final int kOperatorControllerPort = 0;
  }

  public static class Driver {
    public static final int kDriverControllerPort = 1;
  }

  public static class ScoringLocation{
    public static final Translation2d kHubPosition = new Translation2d(4.5, 4);
    public static final Translation2d kFeedTopPosition    = new Translation2d(3, 5.5);
    public static final Translation2d kFeedBottomPosition = new Translation2d(3, 1.5);
  }

  public static class Turret {
    public static final int kTurretMotorID = 21;
    public static final int kEncoderID = 22;
    public static final double kMagnetOffset = 0.05;
    public static final double kMinAngleDegrees = -257;
    public static final double kMaxAngleDegrees = 103;

    public static final double kGearRatioEncoder = (85.0 / 10.0); // 12.0 50
    public static final double kGearRatioTurretAngleRatio = (50.0 / 12.0) * (85.0 / 10.0);

    public static final double kP = 5.5;
    public static final double kI = 0.0;
    public static final double kD = 0.2;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    public static final double kToleranceDegrees = 2.0;
  

    public static final Translation2d kTurretOffset = new Translation2d(-0.094409, -0.168886); //tune
    public static final Rotation2d rotationCorrectionOffset = Rotation2d.fromDegrees(0);
    public static final double kSpringK = 0;
    public static final double kSpringNeutralAngle = 0;
    public static final double kSpringForce = 5.94;  // lbs cause im dum
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
    public static final double kredGoalY = 4;
    public static final double kredGoalX = 12;
    public static final double kblueGoalY = 4;
    public static final double kblueGoalX = 12;
    public static final Pose2d kredGoalPose = new Pose2d(kredGoalX, kredGoalY, new Rotation2d());
    public static final Pose2d kblueGoalPose = new Pose2d(kblueGoalX, kblueGoalY, new Rotation2d());
    public static final String kCamera1 = "ThriftyCam1.0";
    public static final String kCamera2 = "ThriftyCam2.0";
    public static final String kCamera3 = "ThriftyCam3.0";
    public static final String kCamera4 = "ThriftyCam4.0";
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

    public static final Transform3d kCameraTransfromThriftyCamera1 = new Transform3d(Units.inchesToMeters(-14.554),Units.inchesToMeters(9.034), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(278.3)));
    public static final Transform3d kCameraTransfromThriftyCamera2 = new Transform3d(Units.inchesToMeters(-11.73),Units.inchesToMeters(11.57), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(180)));
    public static final Transform3d kCameraTransfromThriftyCamera3 = new Transform3d(Units.inchesToMeters(-12.75),Units.inchesToMeters(8), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(0)));
    public static final Transform3d kCameraTransfromThriftyCamera4 = new Transform3d(Units.inchesToMeters(-14.4),Units.inchesToMeters(10.76), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(99.7)));

    //NEW COMBINED VISION SUBSYSTEM
  }

  public static class ShooterHood {
    public static final int kHoodMotorID = 26;
    public static final int kEncoderID = 27;

    public static final double kMinAngleDegrees = 0.1;
    public static final double kMaxAngleDegrees = 30;


    public static final double kToleranceDegrees = 0.3;
    public static final double kGearRatioEncoder = (54.0 / 18.0);
    public static final double kGearRatioHoodAngleRatio = (54.0 / 18.0) * (175.0 / 10.0);

    public static final double kP = 3.2;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.1;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.2;


    public static final double kMagnetOffset = 0.01611;
    public static final double kFixedHood = 20;
  }

  public static class Shooter {
    public static final int flywheelLeaderID = 23;
    public static final int flywheelFollower1TopRightID = 24;
    public static final int flywheelFollower2BottonRightID = 25;
    public static final double flywheelGearRatio = 27.0/17.0;//Flyhwheel to Motor  25/17 flyhweel to smal hood wheel
    public static final double rpmTolerance = 150.0;

    public static final double kP = 0.075;
    public static final double kI = 0;
    public static final double kD = 0.001;
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double kA = 0.2;
    public static final double[][] distanceTable = {
      {2.01, 1350},
      {2.29, 1400},
      {2.5, 1470},
      {2.8, 1490},
      {3.2, 1500},
      {3.69, 1600},
      {4.5, 1740},
      {5.3, 1800}
    };
    
    public static final double kLatencyCompensation = 0.1; 

    public static final double[][] kTOFTable = {
      { 2.01,  0.38 },
      { 2.29,  0.45 },
      { 2.5,  0.52 },
      { 2.8,  0.60 },
      { 3.2,  0.68 },
      { 4.69,  0.76 },
      { 4.5,  0.85 },
      { 5.3,  0.94 },
    };
  }

  public static class Indexer {
    public static final int m_indexerID = 32;
    public static final double m_indexerGearRatio = 1.0;
    public static final double rpmTolerance = 20;

    public static final double kP = 0.08;
    public static final double kI = 0.01;
    public static final double kD = 0.0005;
    public static final double kS = 0.10;
    public static final double kV = 0.12;
    public static final double kA = 2.0;
    public static final double m_indexerRPM = 3600;

  }

  public static class Spindexer {
    public static final int m_spindexerID = 31;
    public static final double m_spindexerGearRatio = 1.0;
    public static final double rpmTolerance = 20;

    public static final double kP = 0.04;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.10;
    public static final double kV = 0.115;
    public static final double kA = 2.0;
    public static final double m_spindexerRPM = 5800;
  }

  public static class Intake {
    public static final int m_intakeTurretSideLeaderID = 41;
    public static final int m_intakeNonTurretSideFollowerID = 42;

    public static final double rollerPower = 0.1;

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
  
    public static final double kMagnetOffset = 0.19;
    public static final double kGearRatioEncoder = 23.4/1.0;
    public static final double kGearRatioWristAngleRatio = 46.8/1.0;//84/20 78/14 78/20 46.8/1 Motor to Encoder 23.4/1 Encoder to shaft 2/1
    public static final double kAbsoluteGearRatio = 2.0/1.0;
    public static final double kToleranceDegrees = 4.0;
    public static final double kSlewRate = 0.2;


    public static final double kMinAngleDegrees = 0.0;
    public static final double kMaxAngleDegrees = 95.0;


    public static final double kP = 30.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;
  }

  public static final class Hopper {
    public static final int kCANRangeID = 51; 

    public static final double kMaxDistanceMeters = 0.50;
    public static final double kMinDistanceMeters = 0.05; 
    public static final double kFullThresholdPercentage = 90.0;
    public static final double kEmptyThresholdPercentage = 5.0;
  }

  public static final class InSpinShoot {    
    //auto 
    public static final double kIndexerRPM_right = 3600;
    public static final double kSpindexerRPM_right = 5800;
    public static final double kShooterRPM_right = 3000;
    public static final double kHoodAngle_right = 30;

  }
}
