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
    public static final Translation2d kHubPosition = new Translation2d(4.63, 4.035);
    public static final Translation2d kFeedTopPosition    = new Translation2d(3, 6);
    public static final Translation2d kFeedBottomPosition = new Translation2d(3, 2);
  }

  public static class Turret {
    public static final int kTurretMotorID = 21;
    public static final int kEncoderID = 22;
    public static final double kMagnetOffset = 0.2829;
    public static final double kMinAngleDegrees = -180;
    public static final double kMaxAngleDegrees = 180;

    public static final double kGearRatioEncoder = (85.0 / 10.0); // 12.0 50
    public static final double kGearRatioTurretAngleRatio = (50.0 / 12.0) * (85.0 / 10.0);

    public static final double kP = 6.2;
    public static final double kI = 0.0;
    public static final double kD = 0.22;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    public static final double kToleranceDegrees = 5.0;
  

    public static final Translation2d kTurretOffset = new Translation2d(-0.094409, -0.168886); //tune
    public static final double rotationCorrectionOffset = 177.5; //lower is to the left higher is to the right
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
  
    public static final Translation3d kCameraOffset = new Translation3d();

    public static final boolean kExtraVisionDebugInfo = false;

    public static final Transform3d kCameraTransfromThriftyCamera1 = new Transform3d(Units.inchesToMeters(-9.034),Units.inchesToMeters(-14.554), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(278.3)));
    public static final Transform3d kCameraTransfromThriftyCamera2 = new Transform3d(Units.inchesToMeters(-11.57),Units.inchesToMeters(-11.73), Units.inchesToMeters(13.72), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(180)));
    public static final Transform3d kCameraTransfromThriftyCamera3 = new Transform3d(Units.inchesToMeters(-8),Units.inchesToMeters(12.75), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-23), Math.toRadians(0)));
    public static final Transform3d kCameraTransfromThriftyCamera4 = new Transform3d(Units.inchesToMeters(-10.76),Units.inchesToMeters(14.4), Units.inchesToMeters(20.25), new Rotation3d(0,Math.toRadians(-18), Math.toRadians(99.7)));

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


    public static final double kMagnetOffset = 0.442;
    public static final double kFixedHood = 20;

    public static final double[][] distanceTable = {
      {1.15, 5},
      {1.4, 10},
      {1.7, 10},
      {2.12, 15},
      {2.37, 15},
      {2.7, 20},
      {3.0, 20},
      {3.5, 20},
      {4.0, 20},
      {4.5, 20},
      {5.0, 20},

      {5.5, 20},//linear fit data
      {6.0, 20},
      {6.5, 20},
      {7.0, 20},
      {7.5, 20},
      {8.0, 20},
      {8.5, 20},
      {9.0, 20},
      {9.5, 20},
      {10.0, 20},
      {10.5, 20},
      {11.0, 20},
      {11.5, 28},
      {12.0, 28},
      {12.5, 28},
      {13.0, 28},
      {13.5, 28},
      {14.0, 28},
      {14.5, 28},
      {15.0, 28},
      {15.5, 28},
      {16.0, 28},
      {16.5, 28},
      {17.0, 28},
    }; 
  }

  public static class Shooter {
    public static final int flywheelLeaderID = 25;
    public static final int flywheelFollower1TopLeftID = 24;
    public static final int flywheelFollower2BottonLeftID = 23;
    public static final double flywheelGearRatio = 27.0/17.0;//Flyhwheel to Motor  25/17 flyhweel to smal hood wheel
    public static final double rpmTolerance = 150.0;

    public static final double farFeedingRPMConstant = 467.0;

    public static final double kP = 0.075;
    public static final double kI = 0;
    public static final double kD = 0.001;
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double kA = 0.2;
    public static final double[][] distanceTable = {
      {1.15, 1250},
      {1.4, 1280},
      {1.7, 1310},
      {2.12, 1330},
      {2.37, 1375},
      {2.7, 1400},
      {3.0, 1430},
      {3.5, 1510}, 
      {4.0, 1610},
      {4.5, 1690},
      {5, 1770},

      {5.5, 1850.23},//linear fit data
      {6.0, 1931.85},
      {6.5, 2013.57},
      {7.0, 2095.09},
      {7.5, 2176.71},
      {8.0, 2258.33+farFeedingRPMConstant},
      {8.5, 2339.95+farFeedingRPMConstant},
      {9.0, 2421.57+farFeedingRPMConstant},
      {9.5, 2503.19+farFeedingRPMConstant},
      {10.0, 2584.81+farFeedingRPMConstant},
      {10.5, 2666.43+farFeedingRPMConstant},
      {11.0, 2748.06+farFeedingRPMConstant},
      {11.5, 2829.68+farFeedingRPMConstant},
      {12.0, 2911.30+farFeedingRPMConstant},
      {12.5, 2992.92+farFeedingRPMConstant},
      {13.0, 3074.54+farFeedingRPMConstant},
      {13.5, 3156.16+farFeedingRPMConstant},
      {14.0, 3237.77+farFeedingRPMConstant},
      {14.5, 3319.39+farFeedingRPMConstant},
      {15.0, 3401.01+farFeedingRPMConstant},
      {15.5, 3482.63+farFeedingRPMConstant},
      {16.0, 3564.26+farFeedingRPMConstant},
      {16.5, 3645.88+farFeedingRPMConstant},
      {17.0, 3727.50+farFeedingRPMConstant},
    }; //finished


      

    public static final double[][] kTOFTable = {
      {1.15, 0.66},
      {1.4, 0.7},
      {1.7, 0.86},
      {2.12, 0.8},
      {2.37, 0.84},
      {2.7, 0.81},
      {3.0, 0.91},
      {3.5, 1.00},
      {4.0, 1.10},
      {4.5, 1.19},
      {5.0, 1.27},

      {5.5, 1.38},//linear fit data
      {6.0, 1.47},
      {6.5, 1.57},
      {7.0, 1.67},
      {7.5, 1.77},
      {8.0, 1.869},
      {8.5, 1.967},
      {9.0, 2.064},
      {9.5, 2.162},
      {10.0, 2.259},
      {10.5, 2.357},
      {11.0, 2.454},
      {11.5, 2.552},
      {12.0, 2.650},
      {12.5, 2.747},
      {13.0, 2.845},
      {13.5, 2.942},
      {14.0, 3.040},
      {14.5, 3.137},
      {15.0, 3.235},
      {15.5, 3.333},
      {16.0, 3.430},
      {16.5, 3.528},
      {17.0, 3.625},

    };//finished 
    public static final double latency = 0.2;
    public static final double kBackupTime = 0.3;
    public static final double maxAllowedTranslationAccel = 1000; // Untuned
    public static final double maxAllowedRotationalAccel = 1000; // Untuned
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
    public static final double m_indexerRPM = 3877 ;

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
  
    public static final double kMagnetOffset = -0.66;

    


    
    public static final double kGearRatioEncoder = 23.4/1.0;
    public static final double kGearRatioWristAngleRatio = 46.8/1.0;//84/20 78/14 78/20 46.8/1 Motor to Encoder 23.4/1 Encoder to shaft 2/1
    public static final double kAbsoluteGearRatio = 2.0/1.0;
    public static final double kToleranceRotations = 0.1;
    public static final double kSlewRate = 0.2;


    public static final double kMinRotations = 0.0;
    public static final double kMaxRotations = 0.50;

    public static final double downPos = 0.01;
    public static final double upPos = 0.4;

    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.45;
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

  public static final class Autos {

    public static final double TestingShootTimeout = 4.0;

  }
}
