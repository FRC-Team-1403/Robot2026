// package team1403.robot.commands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import team1403.robot.Constants;
// import team1403.robot.subsystems.Indexer;
// import team1403.robot.subsystems.Shooter;
// import team1403.robot.subsystems.ShooterHood;
// import team1403.robot.subsystems.Spindexer;
// import team1403.robot.util.FieldZoneUtil;
// import team1403.robot.util.FieldZoneUtil.Zone;
// import team1403.robot.vision.Vision;


//IF SOTM doesn't work 
// public class ShooterCommand extends Command {
//   private final Shooter m_shooter;
//   private final ShooterHood m_shooterHood;
//   private final Indexer m_indexer;
//   private final Spindexer m_spindexer;
//   private final Vision m_vision;

//   public ShooterCommand(
//       Shooter m_shooter,
//       Indexer m_indexer,
//       Spindexer m_spindexer,
//       ShooterHood m_shooterHood,
//       Vision m_vision) {
//     this.m_shooter = m_shooter;
//     this.m_shooterHood = m_shooterHood;
//     this.m_vision = m_vision;
//     this.m_indexer = m_indexer;
//     this.m_spindexer = m_spindexer;
//     addRequirements(m_shooter, m_indexer, m_spindexer, m_shooterHood);
//   }

//   @Override
//   public void execute() {
//     if (m_vision.hasPose()) {
//       Pose2d robotPose = m_vision.getPose2d();
//       Zone zone = FieldZoneUtil.getZone(robotPose);

//       double deltaX = Constants.Vision.kGoalX - robotPose.getX();
//       double deltaY = Constants.Vision.kGoalY - robotPose.getY();
//       double dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

//       switch (zone) {
//         case MY_ALLIANCE:
//           if (dist > Constants.ShooterTuning.myAllianceZone.range8Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range8ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range8IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range8SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range8HoodAngle);
//           } else if (dist > Constants.ShooterTuning.myAllianceZone.range7Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range7ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range7IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range7SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range7HoodAngle);
//           } else if (dist > Constants.ShooterTuning.myAllianceZone.range6Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range6ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range6IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range6SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range6HoodAngle);
//           } else if (dist > Constants.ShooterTuning.myAllianceZone.range5Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range5ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range5IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range5SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range5HoodAngle);
//           } else if (dist > Constants.ShooterTuning.myAllianceZone.range4Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range4ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range4IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range4SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range4HoodAngle);
//           } else if (dist > Constants.ShooterTuning.myAllianceZone.range3Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range3ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range3IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range3SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range3HoodAngle);
//           } else if (dist > Constants.ShooterTuning.myAllianceZone.range2Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range2ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range2IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range2SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range2HoodAngle);
//           } else {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.myAllianceZone.range1ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range1IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range1SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.myAllianceZone.range1HoodAngle);
//           }

//           break;

//         case NEUTRAL:
//           if (dist > Constants.ShooterTuning.neutralZone.range8Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range8ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range8IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range8SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range8HoodAngle);
//           } else if (dist > Constants.ShooterTuning.neutralZone.range7Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range7ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range7IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range7SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range7HoodAngle);
//           } else if (dist > Constants.ShooterTuning.neutralZone.range6Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range6ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range6IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range6SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range6HoodAngle);
//           } else if (dist > Constants.ShooterTuning.neutralZone.range5Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range5ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range5IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range5SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range5HoodAngle);
//           } else if (dist > Constants.ShooterTuning.neutralZone.range4Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range4ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range4IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range4SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range4HoodAngle);
//           } else if (dist > Constants.ShooterTuning.neutralZone.range3Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range3ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range3IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range3SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range3HoodAngle);
//           } else if (dist > Constants.ShooterTuning.neutralZone.range2Distance) {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range2ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range2IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range2SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range2HoodAngle);
//           } else {
//             m_shooter.setFlywheelTargetRPM(Constants.ShooterTuning.neutralZone.range1ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range1IndexerRpm);
//             m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range1SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.neutralZone.range1HoodAngle);
//           }

//           break;

//         case OPPOSING_ALLIANCE:
//           if (dist > Constants.ShooterTuning.opposingAllianceZone.range8Distance) {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range8ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range8IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range8SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range8HoodAngle);
//           } else if (dist > Constants.ShooterTuning.opposingAllianceZone.range7Distance) {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range7ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range7IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range7SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range7HoodAngle);
//           } else if (dist > Constants.ShooterTuning.opposingAllianceZone.range6Distance) {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range6ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range6IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range6SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range6HoodAngle);
//           } else if (dist > Constants.ShooterTuning.opposingAllianceZone.range5Distance) {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range5ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range5IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range5SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range5HoodAngle);
//           } else if (dist > Constants.ShooterTuning.opposingAllianceZone.range4Distance) {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range4ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range4IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range4SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range4HoodAngle);
//           } else if (dist > Constants.ShooterTuning.opposingAllianceZone.range3Distance) {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range3ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range3IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range3SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range3HoodAngle);
//           } else if (dist > Constants.ShooterTuning.opposingAllianceZone.range2Distance) {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range2ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range2IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range2SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range2HoodAngle);
//           } else {
//             m_shooter.setFlywheelTargetRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range1ShooterRpm);
//             m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range1IndexerRpm);
//             m_spindexer.setSpindexerRPM(
//                 Constants.ShooterTuning.opposingAllianceZone.range1SpindexerRpm);
//             m_shooterHood.setSetpoint(Constants.ShooterTuning.opposingAllianceZone.range1HoodAngle);
//           }

//           break;
//       }
//     }
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
