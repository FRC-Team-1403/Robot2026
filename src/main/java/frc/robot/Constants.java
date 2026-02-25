package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Indexer {
    public static final int rpmTolerance = 30;
    public static final double gearRatio = 1.0 / 1.0;//2.27777
  }

  public static class Spindexer {
    public static final int rpmTolerance = 30;
    public static final double gearRatio = 1.0 / 1.0;//0.6666;
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