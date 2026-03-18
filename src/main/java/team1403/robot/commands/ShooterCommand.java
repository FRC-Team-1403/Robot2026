package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Zone;

import java.util.function.Supplier;

/**
 * This command aims and fires at the correct target while the robot is NOT moving.
 * It uses a fused Pose2d (vision + odometry) for the robot's position, which is more
 * accurate than either source alone.
 *
 * Based on which zone the robot is in, it picks the right target (Hub or Feed station)
 * and the right lookup table of shooter settings. It then interpolates flywheel RPM
 * and hood angle from that table for the measured distance.
 */
public class ShooterCommand extends Command {

  // Physical hood limits — all hood setpoints are clamped to this range to protect the mechanism.
  private static final double kHoodMinDeg = 0.0;
  private static final double kHoodMaxDeg = 30.0;

  // The flywheel must be within this many RPM of its target before we allow feeding.
  // Prevents shooting a ball with too little spin, which would miss the target.
  private static final double kRpmReadyTolerance = 90.0;

  // LOOKUP TABLES  [distance (m), shooter RPM, hood angle (deg)]
  // There is one table per field zone. Each row is one distance breakpoint.
  // When the measured distance falls between two rows, we interpolate between them.
  // Values come from Constants so tuning only requires changing Constants — no
  // changes to this file needed.
  // Indexer and spindexer speeds are NOT in these tables — they are constant per zone
  // and are looked up directly from Constants in the firing gate below.
  private static final double[][] myAllianceTable = {
    {Constants.ShooterTuning.myAllianceZone.range1Distance, Constants.ShooterTuning.myAllianceZone.range1ShooterRpm, Constants.ShooterTuning.myAllianceZone.range1HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range2Distance, Constants.ShooterTuning.myAllianceZone.range2ShooterRpm, Constants.ShooterTuning.myAllianceZone.range2HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range3Distance, Constants.ShooterTuning.myAllianceZone.range3ShooterRpm, Constants.ShooterTuning.myAllianceZone.range3HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range4Distance, Constants.ShooterTuning.myAllianceZone.range4ShooterRpm, Constants.ShooterTuning.myAllianceZone.range4HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range5Distance, Constants.ShooterTuning.myAllianceZone.range5ShooterRpm, Constants.ShooterTuning.myAllianceZone.range5HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range6Distance, Constants.ShooterTuning.myAllianceZone.range6ShooterRpm, Constants.ShooterTuning.myAllianceZone.range6HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range7Distance, Constants.ShooterTuning.myAllianceZone.range7ShooterRpm, Constants.ShooterTuning.myAllianceZone.range7HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range8Distance, Constants.ShooterTuning.myAllianceZone.range8ShooterRpm, Constants.ShooterTuning.myAllianceZone.range8HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range9Distance, Constants.ShooterTuning.myAllianceZone.range9ShooterRpm, Constants.ShooterTuning.myAllianceZone.range9HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range10Distance, Constants.ShooterTuning.myAllianceZone.range10ShooterRpm, Constants.ShooterTuning.myAllianceZone.range10HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range11Distance, Constants.ShooterTuning.myAllianceZone.range11ShooterRpm, Constants.ShooterTuning.myAllianceZone.range11HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range12Distance, Constants.ShooterTuning.myAllianceZone.range12ShooterRpm, Constants.ShooterTuning.myAllianceZone.range12HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range13Distance, Constants.ShooterTuning.myAllianceZone.range13ShooterRpm, Constants.ShooterTuning.myAllianceZone.range13HoodAngle},
    {Constants.ShooterTuning.myAllianceZone.range14Distance, Constants.ShooterTuning.myAllianceZone.range14ShooterRpm, Constants.ShooterTuning.myAllianceZone.range14HoodAngle},
    
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
    {Constants.ShooterTuning.neutralZone.range9Distance, Constants.ShooterTuning.neutralZone.range9ShooterRpm, Constants.ShooterTuning.neutralZone.range9HoodAngle},
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

  private final Shooter m_shooter;
  private final ShooterHood m_shooterHood;
  private final Indexer m_indexer;
  private final Spindexer m_spindexer;
  private final Turret m_turret;
  private final Supplier<Pose2d> m_poseSupplier;

  // Tracks which lookup table is active this loop so we use the right RPM/hood values.
  private double[][] m_currentTable = myAllianceTable;

  // True when the indexer and spindexer are actively feeding.
  // Both always start and stop together — neither runs without the other.
  // We only call set/stop on state transitions, not every 20ms loop.
  private boolean m_isFeeding = false;

  public ShooterCommand(
      Shooter shooter,
      Indexer indexer,
      Spindexer spindexer,
      ShooterHood shooterHood,
      Turret turret,
      Supplier<Pose2d> poseSupplier) {
    m_shooter = shooter;
    m_shooterHood = shooterHood;
    m_indexer = indexer;
    m_spindexer = spindexer;
    m_turret = turret;
    m_poseSupplier = poseSupplier;
    addRequirements(shooter, indexer, spindexer, shooterHood);
  }

  // Called once when the command starts. Stop both feeder motors in case they were
  // left on from a previous command. The flywheel is left alone so if it was already
  // spinning we do not waste time waiting for a full re-spin-up.
  @Override
  public void initialize() {
    m_indexer.stop();
    m_spindexer.stop();
    m_isFeeding = false;
  }

  @Override
  public void execute() {
    Pose2d pose = m_poseSupplier.get();

    // The field is divided into zones. Which zone the robot is in determines
    // which target we aim at (hub vs feed station) and which RPM/hood table we use.
    selectTableFromZone(pose);

    // turret.getDistanceToTarget() returns the distance from the turret pivot
    // (not the robot center) to whatever target is active for the current zone.
    // Using the pivot point rather than the robot center gives a more accurate
    // distance since the turret is physically offset from the robot's center.
    double distance = m_turret.getDistanceToTarget(pose);

    // If the distance is suspiciously small, the pose estimator is likely giving
    // bad data. Skip this loop entirely to avoid sending garbage values to hardware.
    if (distance < 0.3) {
      Logger.recordOutput("Shooter/Warning", "Distance < 0.3m — skipping loop, check pose estimator");
      return;
    }

    
    // Look up the two nearest distance entries in the active table and linearly
    // interpolate between them. Only RPM and hood angle come out of this step —
    // indexer and spindexer speeds are fixed per zone and are set in the firing gate.
    // Returns [flywheelRpm, hoodAngleDeg].
    double[] params = interpolate(m_currentTable, distance);
    double targetRpm = params[0];
    double targetHoodAngle = MathUtil.clamp(params[1], kHoodMinDeg, kHoodMaxDeg);

    // Only these two values change with distance. The indexer and spindexer are
    // intentionally not touched here — they are gated by the readiness check below.
    m_shooter.setFlywheelTargetRPM(targetRpm);
    m_shooterHood.setSetpoint(targetHoodAngle);

    // We only feed a ball once both the flywheel and hood are at their targets.
    // The indexer and spindexer run at fixed speeds that come from Constants and
    // do not change with distance — only which zone's constant is used changes.
    // Both feeders start and stop together as a pair, never one without the other.
    // The m_isFeeding flag means we only call motor methods on the transition
    // in or out, not every 20ms loop, which avoids unnecessary CAN traffic.
    boolean shooterReady = Math.abs(m_shooter.getFlywheelLeaderRPM() - targetRpm) < kRpmReadyTolerance;
    boolean hoodReady    = m_shooterHood.atSetpoint();
    boolean readyToFire  = shooterReady && hoodReady;

    if (readyToFire && !m_isFeeding) {
      if (m_currentTable == myAllianceTable) {
        m_indexer.setIndexerRPM(Constants.ShooterTuning.myAllianceZone.range1IndexerRpm);
        m_spindexer.setSpindexerRPM(Constants.ShooterTuning.myAllianceZone.range1SpindexerRpm);
      } else if (m_currentTable == neutralTable) {
        m_indexer.setIndexerRPM(Constants.ShooterTuning.neutralZone.range1IndexerRpm);
        m_spindexer.setSpindexerRPM(Constants.ShooterTuning.neutralZone.range1SpindexerRpm);
      } else {
        m_indexer.setIndexerRPM(Constants.ShooterTuning.opposingAllianceZone.range1IndexerRpm);
        m_spindexer.setSpindexerRPM(Constants.ShooterTuning.opposingAllianceZone.range1SpindexerRpm);
      }
      m_isFeeding = true;
    } else if (!readyToFire && m_isFeeding) {
      m_indexer.stop();
      m_spindexer.stop();
      m_isFeeding = false;
    }

    
    String zoneName;
    if (m_currentTable == myAllianceTable) {
      zoneName = "MY_ALLIANCE";
    } else if (m_currentTable == neutralTable) {
      zoneName = "NEUTRAL";
    } else {
      zoneName = "OPPOSING";
    }

    Logger.recordOutput("Shooter/Distance",     distance);
    Logger.recordOutput("Shooter/TargetRPM",    targetRpm);
    Logger.recordOutput("Shooter/HoodAngle",    targetHoodAngle);
    Logger.recordOutput("Shooter/ShooterReady", shooterReady);
    Logger.recordOutput("Shooter/HoodReady",    hoodReady);
    Logger.recordOutput("Shooter/IsFeeding",    m_isFeeding);
    Logger.recordOutput("Shooter/Zone",         zoneName);
  }


  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_indexer.stop();
    m_spindexer.stop();
    m_shooterHood.setSetpoint(0);
    m_isFeeding = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }


  /**
   * Sets m_currentTable based on which zone the robot is in.
   * MY_ALLIANCE uses the hub shot table.
   * NEUTRAL and OPPOSING use their respective feed shot tables.
   * The actual target position is handled inside turret.getDistanceToTarget()
   * via Blackbox, so we only need to pick the right RPM/hood table here.
   */
  private void selectTableFromZone(Pose2d pose) {
    Zone zone = FieldZoneUtil.getZone(pose);
    if (zone == Zone.MY_ALLIANCE) {
      m_currentTable = myAllianceTable;
    } else if (zone == Zone.NEUTRAL) {
      m_currentTable = neutralTable;
    } else {
      m_currentTable = opposingAllianceTable;
    }
  }

  /**
   * Linearly interpolates flywheel RPM and hood angle for a given distance between
   * two entries in the lookup table. If the distance is below the first entry or above
   * the last entry, the nearest edge value is returned as-is (no extrapolation).
   * Returns a two-element array: [flywheelRpm, hoodAngleDeg].
   */
  private static double[] interpolate(double[][] table, double dist) {
    if (dist <= table[0][0]) {
      return new double[]{table[0][1], table[0][2]};
    }

    if (dist >= table[table.length - 1][0]) {
      return new double[]{table[table.length - 1][1], table[table.length - 1][2]};
    }

    for (int i = 0; i < table.length - 1; i++) {
      if (dist <= table[i + 1][0]) {
        double t         = (dist - table[i][0]) / (table[i + 1][0] - table[i][0]);
        double rpm       = table[i][1] + t * (table[i + 1][1] - table[i][1]);
        double hoodAngle = table[i][2] + t * (table[i + 1][2] - table[i][2]);
        return new double[]{rpm, hoodAngle};
      }
    }

    return new double[]{table[table.length - 1][1], table[table.length - 1][2]};
  }
}