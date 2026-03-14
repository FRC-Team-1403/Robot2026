package team1403.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Zone;
import team1403.robot.vision.Vision;

public class ShooterCommand extends Command {
  private final Shooter m_shooter;
  private final ShooterHood m_shooterHood;
  private final Indexer m_indexer;
  private final Spindexer m_spindexer;
  private final Vision m_vision;

  private static final double[][] myAllianceTable = {
    {Constants.ShooterTuning.myAllianceZone.range1Distance, Constants.ShooterTuning.myAllianceZone.range1ShooterRpm, Constants.ShooterTuning.myAllianceZone.range1HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range2Distance, Constants.ShooterTuning.myAllianceZone.range2ShooterRpm, Constants.ShooterTuning.myAllianceZone.range2HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range3Distance, Constants.ShooterTuning.myAllianceZone.range3ShooterRpm, Constants.ShooterTuning.myAllianceZone.range3HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range4Distance, Constants.ShooterTuning.myAllianceZone.range4ShooterRpm, Constants.ShooterTuning.myAllianceZone.range4HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range5Distance, Constants.ShooterTuning.myAllianceZone.range5ShooterRpm, Constants.ShooterTuning.myAllianceZone.range5HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range6Distance, Constants.ShooterTuning.myAllianceZone.range6ShooterRpm, Constants.ShooterTuning.myAllianceZone.range6HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range7Distance, Constants.ShooterTuning.myAllianceZone.range7ShooterRpm, Constants.ShooterTuning.myAllianceZone.range7HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range8Distance, Constants.ShooterTuning.myAllianceZone.range8ShooterRpm, Constants.ShooterTuning.myAllianceZone.range8HoodAngle},
  };

  private static final double[][] neutralTable = {
    {Constants.ShooterTuning.neutralZone.range1Distance, Constants.ShooterTuning.neutralZone.range1ShooterRpm, Constants.ShooterTuning.neutralZone.range1HoodAngle},
    {Constants.ShooterTuning.neutralZone.range2Distance, Constants.ShooterTuning.neutralZone.range2ShooterRpm, Constants.ShooterTuning.neutralZone.range2HoodAngle},
    {Constants.ShooterTuning.neutralZone.range3Distance, Constants.ShooterTuning.neutralZone.range3ShooterRpm, Constants.ShooterTuning.neutralZone.range3HoodAngle},
    {Constants.ShooterTuning.neutralZone.range4Distance, Constants.ShooterTuning.neutralZone.range4ShooterRpm, Constants.ShooterTuning.neutralZone.range4HoodAngle},
    {Constants.ShooterTuning.neutralZone.range5Distance, Constants.ShooterTuning.neutralZone.range5ShooterRpm, Constants.ShooterTuning.neutralZone.range5HoodAngle},
    {Constants.ShooterTuning.neutralZone.range6Distance, Constants.ShooterTuning.neutralZone.range6ShooterRpm, Constants.ShooterTuning.neutralZone.range6HoodAngle},
    {Constants.ShooterTuning.neutralZone.range7Distance, Constants.ShooterTuning.neutralZone.range7ShooterRpm, Constants.ShooterTuning.neutralZone.range7HoodAngle},
    {Constants.ShooterTuning.neutralZone.range8Distance, Constants.ShooterTuning.neutralZone.range8ShooterRpm, Constants.ShooterTuning.neutralZone.range8HoodAngle},
  };

  private static final double[][] opposingAllianceTable = {
    {Constants.ShooterTuning.opposingAllianceZone.range1Distance, Constants.ShooterTuning.opposingAllianceZone.range1ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range1HoodAngle},
    {Constants.ShooterTuning.opposingAllianceZone.range2Distance, Constants.ShooterTuning.opposingAllianceZone.range2ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range2HoodAngle},
    {Constants.ShooterTuning.opposingAllianceZone.range3Distance, Constants.ShooterTuning.opposingAllianceZone.range3ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range3HoodAngle},
    {Constants.ShooterTuning.opposingAllianceZone.range4Distance, Constants.ShooterTuning.opposingAllianceZone.range4ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range4HoodAngle},
    {Constants.ShooterTuning.opposingAllianceZone.range5Distance, Constants.ShooterTuning.opposingAllianceZone.range5ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range5HoodAngle},
    {Constants.ShooterTuning.opposingAllianceZone.range6Distance, Constants.ShooterTuning.opposingAllianceZone.range6ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range6HoodAngle},
    {Constants.ShooterTuning.opposingAllianceZone.range7Distance, Constants.ShooterTuning.opposingAllianceZone.range7ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range7HoodAngle},
    {Constants.ShooterTuning.opposingAllianceZone.range8Distance, Constants.ShooterTuning.opposingAllianceZone.range8ShooterRpm, Constants.ShooterTuning.opposingAllianceZone.range8HoodAngle},
  };

  public ShooterCommand(
      Shooter m_shooter,
      Indexer m_indexer,
      Spindexer m_spindexer,
      ShooterHood m_shooterHood,
      Vision m_vision) {
    this.m_shooter = m_shooter;
    this.m_shooterHood = m_shooterHood;
    this.m_vision = m_vision;
    this.m_indexer = m_indexer;
    this.m_spindexer = m_spindexer;
    addRequirements(m_shooter, m_indexer, m_spindexer, m_shooterHood);
  }

  private double[] interpolate(double[][] table, double dist) {
    if (dist <= table[0][0]) return new double[]{table[0][1], table[0][2]};
    if (dist >= table[table.length - 1][0]) return new double[]{table[table.length - 1][1], table[table.length - 1][2]};
    for (int i = 0; i < table.length - 1; i++) {
      if (dist <= table[i + 1][0]) {
        double t = (dist - table[i][0]) / (table[i + 1][0] - table[i][0]);
        return new double[]{
          table[i][1] + t * (table[i + 1][1] - table[i][1]),
          table[i][2] + t * (table[i + 1][2] - table[i][2])
        };
      }
    }
    return new double[]{table[table.length - 1][1], table[table.length - 1][2]};
  }

  @Override
  public void execute() {
    if (m_vision.hasPose()) {
      Pose2d robotPose = m_vision.getPose2d();
      Zone zone = FieldZoneUtil.getZone(robotPose);
      double deltaX = Constants.Vision.kGoalX - robotPose.getX();
      double deltaY = Constants.Vision.kGoalY - robotPose.getY();
      double dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
      switch (zone) {
        case MY_ALLIANCE:
          double[] myVals = interpolate(myAllianceTable, dist);
          m_shooter.setFlywheelTargetRPM(myVals[0]);
          m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range1IndexerRpm);
          m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range1SpindexerRpm);
          m_shooterHood.setSetpoint(myVals[1]);
          break;
        case NEUTRAL:
          double[] neutralVals = interpolate(neutralTable, dist);
          m_shooter.setFlywheelTargetRPM(neutralVals[0]);
          m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range1IndexerRpm);
          m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range1SpindexerRpm);
          m_shooterHood.setSetpoint(neutralVals[1]);
          break;
        case OPPOSING_ALLIANCE:
          double[] oppVals = interpolate(opposingAllianceTable, dist);
          m_shooter.setFlywheelTargetRPM(oppVals[0]);
          m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range1IndexerRpm);
          m_spindexer.setSpindexerRPM(Constants.ShooterTuning.opposingAllianceZone.range1SpindexerRpm);
          m_shooterHood.setSetpoint(oppVals[1]);
          break;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}