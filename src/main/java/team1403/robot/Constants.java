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
  public static final double brownOutVoltage = 8.0;
public static final double minimumBatteryVoltage = 11.0;


  public static class Operator {
    public static final int kOperatorControllerPort = 0;
  }

  public static class Driver {
    public static final int kDriverControllerPort = 1;
  }

  public static class ScoringLocation{
    //4.63 4.035
    public static final Translation2d kHubPosition = new Translation2d(4.63, 4.035);
    public static final Translation2d kFeedTopPosition    = new Translation2d(3, 6.5);
    public static final Translation2d kFeedBottomPosition = new Translation2d(3, 1.5);
  }

  public static class Turret {
    public static final int kTurretMotorID = 21;
    public static final int kEncoderID = 22;
    public static final double kMagnetOffset = -0.27417;
    public static final double kMinAngleDegrees = -162;
    public static final double kMaxAngleDegrees = 198;

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

    //public static final Transform3d kCameraTransfromThriftyCamera1 = new Transform3d(Units.inchesToMeters(-14.554),Units.inchesToMeters(-9.034), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(278.3)));
    //public static final Transform3d kCameraTransfromThriftyCamera2 = new Transform3d(Units.inchesToMeters(-11.73),Units.inchesToMeters(-11.57), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(180)));
    //public static final Transform3d kCameraTransfromThriftyCamera3 = new Transform3d(Units.inchesToMeters(-12.75),Units.inchesToMeters(8), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(0)));
    //public static final Transform3d kCameraTransfromThriftyCamera4 = new Transform3d(Units.inchesToMeters(-14.4),Units.inchesToMeters(10.76), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(99.7)));

    public static final Transform3d kCameraTransfromThriftyCamera1 = new Transform3d(Units.inchesToMeters(-9.034),Units.inchesToMeters(-14.554), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(278.3)));
    public static final Transform3d kCameraTransfromThriftyCamera2 = new Transform3d(Units.inchesToMeters(-11.57),Units.inchesToMeters(-11.73), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(180)));
    public static final Transform3d kCameraTransfromThriftyCamera3 = new Transform3d(Units.inchesToMeters(-8),Units.inchesToMeters(12.75), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(0)));
    public static final Transform3d kCameraTransfromThriftyCamera4 = new Transform3d(Units.inchesToMeters(-10.76),Units.inchesToMeters(14.4), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(99.7)));

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

    public static final double[][] distanceTable = {
      {1.15, 5},
      {1.4, 10},
      {1.7, 10},
      {2.12, 15},
      {2.37, 15},
      {2.7, 20},
      {3.0, 20},
      {3.3, 20},
      {3.59, 20},
      {3.89, 20},
      {4.22, 20},
      {4.5, 20},
      {4.89, 20},
      {5.2, 20}
    }; //finished 
    //CURVE FITTED TABLE
    // public static final double[][] distanceTable = {
    //   {1.15, 5},
    //   {1.4, 10},
    //   {1.7, 10},
    //   {2.12, 15},
    //   {2.37, 15},
    //   {2.7, 20},
    //   {3, 20},
    //   {3.3, 20},
    //   {3.59, 20},
    //   {3.89, 20},
    //   {4.22, 20},
    //   {4.5, 20},
    //   {4.89, 20},
    //   {5.2, 20}
    // };
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

    public static final double rpmoffset = 10;

    //REGULAR
    // public static final double[][] distanceTable = {
    //   {1.15, 1250},
    //   {1.4, 1280},
    //   {1.7, 1310},
    //   {2.12, 1330},
    //   {2.37, 1375},
    //   {2.7, 1410},
    //   {3.0, 1430},
    //   {3.3, 1490}, 
    //   {3.59, 1550}, //big jump
    //   {3.89, 1650},//big jump
    //   {4.22, 1710},
    //   {4.5,1750},
    //   {4.89, 1805},
    //   {5.2, 1835}
    // }; //finished 
      
    public static final double kLatencyCompensation = 0.15; 
    //REGULAR
    // public static final double[][] kTOFTable = {
    //   {1.15, 0.66},
    //   {1.4, 0.7},
    //   {1.7, 0.84},
    //   {2.12, 0.8},
    //   {2.37, 0.84},
    //   {2.7, 0.81},
    //   {3.0, 0.83},
    //   {3.3, 0.86}, 
    //   {3.59, 0.93},
    //   {3.89, 1.05},
    //   {4.22, 1.14},
    //   {4.5, 1.27},
    //   {4.89, 1.31},
    //   {5.2, 1.34}
    // };
    public static final double kBackupTime = 0.2;

    //CURVE FITTED TABLES
    public static final double[][] kTOFTable = {
      {1.15, 0.72},
      {1.4, 0.73},
      {1.7, 0.74},
      {2.12, 0.77},
      {2.37, 0.80},
      {2.7, 0.84},
      {3, 0.88},
      {3.3, 0.93},
      {3.59, 0.98},
      {3.89, 1.04},
      {4.22, 1.12},
      {4.5, 1.19},
      {4.89, 1.29},
      {5.2, 1.39}
    };

    //CURVE
    public static final double[][] distanceTable = {
      {1.15, 1246-rpmoffset},
      {1.4, 1269-rpmoffset},
      {1.7, 1299-rpmoffset},
      {2.12, 1346-rpmoffset},
      {2.37, 1377-rpmoffset},
      {2.7, 1421-rpmoffset},
      {3, 1464-rpmoffset},
      {3.3, 1510-rpmoffset},
      {3.59, 1558-rpmoffset},
      {3.89, 1610-rpmoffset},
      {4.22, 1671-rpmoffset},
      {4.5, 1750-rpmoffset},
      {4.89, 1806-rpmoffset},
      {5.2, 1835-rpmoffset}
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
