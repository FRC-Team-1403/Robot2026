package team1403.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldZoneUtil {
  public static final double kFieldLength = 16.54;
  public static final double kFieldWidth = 8.07;
  public static final double kAllianceZoneDepth = 4.03;

  public enum Zone {
    MY_ALLIANCE,
    NEUTRAL,
    OPPOSING_ALLIANCE
  }

  public enum Side {
    TOP,
    BOTTOM
  }

  public static Zone getZone(Pose2d robotPose) {
    double x = robotPose.getX();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (alliance == Alliance.Blue) {
      if (x <= kAllianceZoneDepth) {
        return Zone.MY_ALLIANCE;
      } else if (x >= (kFieldLength - kAllianceZoneDepth)) {
        return Zone.OPPOSING_ALLIANCE;
      } else {
        return Zone.NEUTRAL;
      }
    } else {
      if (x >= (kFieldLength - kAllianceZoneDepth)) {
        return Zone.MY_ALLIANCE;
      } else if (x <= kAllianceZoneDepth) {
        return Zone.OPPOSING_ALLIANCE;
      } else {
        return Zone.NEUTRAL;
      }
    }
  }

  public static Side getSide(Pose2d robotPose) {
    double midPoint = kFieldWidth / 2.0;

    if (robotPose.getY() >= midPoint) {
      return Side.TOP;
    } else {
      return Side.BOTTOM;
    }
  }
}
