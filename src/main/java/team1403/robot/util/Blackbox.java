package team1403.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team1403.robot.Constants;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Side;
import team1403.robot.util.FieldZoneUtil.Zone;

public class Blackbox {


  public static Translation2d mirrorForAlliance(Translation2d bluePosition) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return new Translation2d(FieldZoneUtil.kFieldLength - bluePosition.getX(), bluePosition.getY());
    }
    return bluePosition;
  }

  public static Translation2d getActiveTarget(Pose2d pose) {
    Zone zone = FieldZoneUtil.getZone(pose);
    Side side = FieldZoneUtil.getSide(pose);
    if (zone == Zone.MY_ALLIANCE) {
      return mirrorForAlliance(Constants.ScoringLocation.kHubPosition);
    } else {
      if (side == Side.TOP) {
        return mirrorForAlliance(Constants.ScoringLocation.kFeedTopPosition);
      } else {
        return mirrorForAlliance(Constants.ScoringLocation.kFeedBottomPosition);
      }
    }
  }

}