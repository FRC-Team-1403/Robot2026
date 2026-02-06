package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Shooter {
    public static final int flywheelLeaderID = 1;
    public static final int flywheelFollowerID = 2;
    public static final int rollerMotorID = 42;

    public static final int rpmTolerance = 30;
    public static final double rollerGearRatio = 34.0 / 24.0;

    public static final double flywheelKT = 0.00812;
    public static final double flywheelMOI = 0.001;

    public static final double flywheelKP = 0.25;
    public static final double flywheelKI = 0.01;
    public static final double flywheelKD = 0.005;
    public static final double flywheelKS = 0.10;
    public static final double flywheelKV = 0.115;
    public static final double flywheelKA = 3.0;

    public static final double flywheelStatorCurrentLimit = 40;
    public static final double flywheelSupplyCurrentLimit = 40;

    public static final double rollerKP = 0.00008;
    public static final double rollerKI = 0.0;
    public static final double rollerKD = 0.0001;
    public static final double rollerKS = 0.0;
    public static final double rollerKV = 0.0021;
    public static final double rollerKA = 0.0;

    public static final double rollerMaxVelocity = 5000;
    public static final double rollerMaxAcceleration = 10000;

    public static final double rollerCurrentLimit = 40;
  }
}