package frc.robot;

public final class Constants {
  public static class Operator {
    public static final int kOperatorControllerPort = 0;
  }

  public static class Driver {
    public static final int kDriverControllerPort = 1;
  }


public static class ShooterHood {
    public static final int kHoodMotorID = 10;
    public static final int kEncoderID = 5;

    public static final double kMinAngleDegrees = 0.1;
    public static final double kMaxAngleDegrees = 30;

    public static final double kGain =4;
    public static final double kMaxSpeed = 12.0;
    public static final double kMinSpeed = 2.0;

    public static final double kToleranceDegrees = 0.3;
    public static final double kGearRatioEncoder = (56.0/16.0);
    public static final double kGearRatioHoodAngleRatio = (56.0/16.0)*(175.0/10.0);


    public static final double kRampUpTime = 0.01;
    public static final double kRampDownTime = 0.001;
    public static final double kLoopTime = 0.02;
    public static final double kUnitsPerRampTime = 100;

    public static final double kS = 0.0;
    public static final double kG = 2.5;
    public static final double kV = 0;
    public static final double kA = 0;

  }

  public static class Shooter {
    public static final int flywheelLeaderID = 1;
    public static final int flywheelFollowerID = 20;
    public static final int flywheelFollower2ID = 30;
    public static final double flywheelGearRatio = 1;
    public static final double rpmTolerance = 20;

    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kS = 0.08;
    public static final double kV = 0.135;
    public static final double kA = 1.0;

  
  }
}

