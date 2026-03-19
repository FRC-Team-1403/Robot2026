package team1403.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldZoneUtil {
  public static final double kFieldLength = 16.54;
  public static final double kFieldWidth = 8.07;
  public static final double kAllianceZoneDepth = 4.03;

  // Width of crossing band (only extends INTO neutral)
  public static final double kCrossingWidth = 0.5;

  public enum Zone {
    MY_ALLIANCE,
    NEUTRAL,
    OPPOSING_ALLIANCE,
    CROSSING
  }

  public enum Side {
    TOP,
    BOTTOM
  }

  public static Zone getZone(Pose2d robotPose) {
    double x = robotPose.getX();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (alliance == Alliance.Blue) {

      // Crossing band: just OUTSIDE my alliance (into neutral)
      if (x >= kAllianceZoneDepth && x <= kAllianceZoneDepth + kCrossingWidth) {
        return Zone.CROSSING;
      }

      // Crossing band: just BEFORE opponent alliance (from neutral)
      if (x <= (kFieldLength - kAllianceZoneDepth) &&
          x >= (kFieldLength - kAllianceZoneDepth - kCrossingWidth)) {
        return Zone.CROSSING;
      }

      if (x < kAllianceZoneDepth) {
        return Zone.MY_ALLIANCE;
      } else if (x > (kFieldLength - kAllianceZoneDepth)) {
        return Zone.OPPOSING_ALLIANCE;
      } else {
        return Zone.NEUTRAL;
      }

    } else { // RED alliance (mirrored)

      // Crossing band: just OUTSIDE my alliance (into neutral)
      if (x <= (kFieldLength - kAllianceZoneDepth) &&
          x >= (kFieldLength - kAllianceZoneDepth - kCrossingWidth)) {
        return Zone.CROSSING;
      }

      // Crossing band: just BEFORE opponent alliance (from neutral)
      if (x >= kAllianceZoneDepth &&
          x <= kAllianceZoneDepth + kCrossingWidth) {
        return Zone.CROSSING;
      }

      if (x > (kFieldLength - kAllianceZoneDepth)) {
        return Zone.MY_ALLIANCE;
      } else if (x < kAllianceZoneDepth) {
        return Zone.OPPOSING_ALLIANCE;
      } else {
        return Zone.NEUTRAL;
      }
    }
  }

  public static Side getSide(Pose2d robotPose) {
    double midPoint = kFieldWidth / 2.0;
    return robotPose.getY() >= midPoint ? Side.TOP : Side.BOTTOM;
  }
}