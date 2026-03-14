package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Side;
import team1403.robot.util.FieldZoneUtil.Zone;

import java.util.function.Supplier;

/**
 * ShooterCommand — Stationary Shooting
 *
 * Aims and fires at the correct target while the robot is NOT moving.
 * Uses a fused Pose2d (vision + odometry) for position — more accurate
 * than vision or odometry alone. Picks Hub or Feed target based on field
 * zone, mirrors for Red alliance, and corrects distance for the turret
 * pivot being offset from the robot center. No shoot-on-move compensation.
 *
 * Only flywheel RPM and hood angle change with distance (interpolated).
 * Indexer and spindexer run at a fixed speed per zone and only activate
 * once the flywheel, turret, and hood are all on target.
 */
public class ShooterCommand extends Command {

  // -------------------------------------------------------------------------
  // FIELD TARGET POSITIONS (Blue alliance coordinates)
  // Automatically mirrored to Red alliance coordinates at runtime.
  // -------------------------------------------------------------------------
  private static final Translation2d hubPosition        = new Translation2d(8.27, 4.105);
  private static final Translation2d feedTopPosition    = new Translation2d(4.03, 6.5);
  private static final Translation2d feedBottomPosition = new Translation2d(4.03, 1.5);

  /**
   * Distance from the robot center to the turret pivot, in robot-relative
   * meters (X = forward, Y = left). Rotated into field-frame each loop so
   * it stays accurate as the robot turns.
   * TODO: Measure from robot and update before competition.
   */
  private static final Translation2d turretOffset = new Translation2d(0.1, 0.05);

  // Physical hood limits — all setpoints are clamped to this range.
  private static final double kHoodMinDeg = 0.0;
  private static final double kHoodMaxDeg = 30.0;

  // All three mechanisms must be within these tolerances before we fire.
  private static final double kRpmReadyTolerance       = 150.0; // RPM
  private static final double kTurretReadyToleranceDeg = 1.0;   // Degrees

  // -------------------------------------------------------------------------
  // LOOKUP TABLES  [distance (m), shooter RPM, hood angle (deg)]
  // One table per field zone. Values loaded from Constants so tuning
  // only requires changing Constants — no recompile of this file needed.
  // Indexer and spindexer are NOT in these tables — they are constant per
  // zone and are looked up directly from Constants when ready to fire.
  // -------------------------------------------------------------------------
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

  private final Shooter          m_shooter;
  private final ShooterHood      m_shooterHood;
  private final Indexer          m_indexer;
  private final Spindexer        m_spindexer;
  private final Turret           m_turret;
  private final Supplier<Pose2d> m_poseSupplier; // Fused pose: vision + odometry combined

  private Translation2d m_currentTarget = hubPosition;
  private double[][]    m_currentTable  = myAllianceTable;

  // Tracks whether the indexer and spindexer are actively feeding.
  // Both always start and stop together — neither runs without the other.
  // Prevents calling set/stop every 20ms when state hasn't changed.
  private boolean m_isFeeding = false;

  /**
   * @param shooter      Flywheel subsystem
   * @param indexer      Ball-feeding indexer subsystem
   * @param spindexer    Ball-staging spindexer subsystem
   * @param shooterHood  Hood angle subsystem
   * @param turret       Turret rotation subsystem
   * @param poseSupplier Fused Pose2d supplier (vision + odometry) from swerve drive
   */
  public ShooterCommand(
      Shooter shooter,
      Indexer indexer,
      Spindexer spindexer,
      ShooterHood shooterHood,
      Turret turret,
      Supplier<Pose2d> poseSupplier) {
    m_shooter      = shooter;
    m_shooterHood  = shooterHood;
    m_indexer      = indexer;
    m_spindexer    = spindexer;
    m_turret       = turret;
    m_poseSupplier = poseSupplier;
    addRequirements(shooter, indexer, spindexer, shooterHood, turret);
  }

  @Override
  public void initialize() {
    // Stop both feeder motors on start. They will only activate together once
    // the flywheel, turret, and hood are all confirmed on target in execute().
    // Flywheel is left alone so a pre-spinning flywheel keeps its speed and
    // avoids a full re-spin-up delay.
    m_indexer.stop();
    m_spindexer.stop();
    m_isFeeding = false;
  }

  @Override
  public void execute() {
    // Get the best available robot position (vision-corrected odometry).
    Pose2d pose = m_poseSupplier.get();

    // -------------------------------------------------------------------------
    // 1. TURRET PIVOT POSITION (Off-Center Correction)
    //    The turret is not at the robot center. Rotate the robot-relative offset
    //    into field coordinates using the robot's heading, then add it to the
    //    robot's field position. All distance math below uses this pivot point
    //    so aiming is accurate regardless of where the turret sits on the robot.
    // -------------------------------------------------------------------------
    Translation2d turretPivotField = pose.getTranslation()
        .plus(turretOffset.rotateBy(pose.getRotation()));

    // -------------------------------------------------------------------------
    // 2. SELECT TARGET & MEASURE DISTANCE
    //    Pick Hub or Feed target based on which zone the robot is in.
    //    Distance is measured from the turret pivot for maximum accuracy.
    // -------------------------------------------------------------------------
    selectTargetFromZone(pose);

    Translation2d toTarget = m_currentTarget.minus(turretPivotField);
    double distance = toTarget.getNorm();

    // If distance is unrealistically small, odometry/vision data is bad.
    // Skip this loop entirely rather than sending garbage to the hardware.
    if (distance < 1.0) {
      Logger.recordOutput("Shooter/Warning", "Distance < 1.0m — skipping loop, check pose estimator");
      return;
    }

    // -------------------------------------------------------------------------
    // 3. INTERPOLATE FLYWHEEL & HOOD SETTINGS
    //    Only flywheel RPM and hood angle change with distance — these are the
    //    only two values interpolated from the table. Indexer and spindexer
    //    speeds are constant per zone and handled separately in the firing gate.
    //    Returns [flywheelRpm, hoodAngleDeg].
    // -------------------------------------------------------------------------
    double[] params        = interpolate(m_currentTable, distance);
    double targetRpm       = params[0];
    double targetHoodAngle = MathUtil.clamp(params[1], kHoodMinDeg, kHoodMaxDeg);

    // -------------------------------------------------------------------------
    // 4. COMPUTE TURRET ANGLE
    //    Find the field-relative angle from the turret pivot to the target,
    //    then subtract the robot's heading to get the turret-relative angle.
    //    Wrap to -180..180 so we never spin past the wire stop.
    // -------------------------------------------------------------------------
    Rotation2d fieldAngle        = toTarget.getAngle();
    double     rawTurretAngle    = fieldAngle.minus(pose.getRotation()).getDegrees();
    double     constrainedTurret = MathUtil.inputModulus(rawTurretAngle, -180, 180);

    // -------------------------------------------------------------------------
    // 5. HARDWARE OUTPUT
    //    Only flywheel, hood, and turret are updated here each loop.
    //    Indexer and spindexer are intentionally left alone — they are
    //    gated by the readiness check below and never touched here.
    // -------------------------------------------------------------------------
    m_shooter.setFlywheelTargetRPM(targetRpm);
    m_shooterHood.setSetpoint(targetHoodAngle);
    m_turret.setSetpoint(constrainedTurret);

    // -------------------------------------------------------------------------
    // 6. FIRING LOGIC
    //    Indexer AND spindexer only activate once all three mechanisms are ready.
    //    Both start and stop together as a pair — neither runs without the other.
    //    Their speeds are fixed per zone from Constants and never change with
    //    distance. We use a state flag so we only call set/stop on transitions,
    //    not every 20ms loop.
    // -------------------------------------------------------------------------
    double  turretError  = MathUtil.inputModulus(m_turret.getTurretAngle() - constrainedTurret, -180, 180);
    boolean turretReady  = Math.abs(turretError) < kTurretReadyToleranceDeg;
    boolean shooterReady = Math.abs(m_shooter.getFlywheelLeaderRPM() - targetRpm) < kRpmReadyTolerance;
    boolean hoodReady    = m_shooterHood.atSetpoint();
    boolean readyToFire  = turretReady && shooterReady && hoodReady;

    if (readyToFire && !m_isFeeding) {
      // All systems on target — start both feeders at their fixed zone speeds
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
      // Something drifted off target — stop both feeders until ready again
      m_indexer.stop();
      m_spindexer.stop();
      m_isFeeding = false;
    }

    // -------------------------------------------------------------------------
    // 7. LOGGING
    // -------------------------------------------------------------------------
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
    Logger.recordOutput("Shooter/TurretAngle",  constrainedTurret);
    Logger.recordOutput("Shooter/TurretPivotX", turretPivotField.getX());
    Logger.recordOutput("Shooter/TurretPivotY", turretPivotField.getY());
    Logger.recordOutput("Shooter/TurretReady",  turretReady);
    Logger.recordOutput("Shooter/ShooterReady", shooterReady);
    Logger.recordOutput("Shooter/HoodReady",    hoodReady);
    Logger.recordOutput("Shooter/IsFeeding",    m_isFeeding);
    Logger.recordOutput("Shooter/Zone",         zoneName);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop everything cleanly when the button is released or command is interrupted.
    m_shooter.stop();
    m_indexer.stop();
    m_spindexer.stop();
    m_turret.stopMotor();
    m_isFeeding = false;
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until the button is released
  }

  // ---------------------------------------------------------------------------
  // HELPERS
  // ---------------------------------------------------------------------------

  /**
   * Sets m_currentTarget and m_currentTable based on which zone the robot is in.
   * MY_ALLIANCE = aim at Hub. NEUTRAL or OPPOSING = aim at nearest Feed station.
   * All positions are mirrored for Red alliance automatically.
   */
  private void selectTargetFromZone(Pose2d pose) {
    Zone zone = FieldZoneUtil.getZone(pose);
    Side side = FieldZoneUtil.getSide(pose);

    if (zone == Zone.MY_ALLIANCE) {
      m_currentTarget = mirrorForAlliance(hubPosition);
      m_currentTable  = myAllianceTable;
    } else if (zone == Zone.NEUTRAL) {
      if (side == Side.TOP) {
        m_currentTarget = mirrorForAlliance(feedTopPosition);
      } else {
        m_currentTarget = mirrorForAlliance(feedBottomPosition);
      }
      m_currentTable = neutralTable;
    } else {
      // OPPOSING_ALLIANCE zone — aim at the nearest feed station
      if (side == Side.TOP) {
        m_currentTarget = mirrorForAlliance(feedTopPosition);
      } else {
        m_currentTarget = mirrorForAlliance(feedBottomPosition);
      }
      m_currentTable = opposingAllianceTable;
    }
  }

  /**
   * Mirrors a Blue-alliance field position to the Red side by flipping its X.
   * Lets us define all targets once and use them for both alliances.
   */
  private static Translation2d mirrorForAlliance(Translation2d bluePosition) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return new Translation2d(FieldZoneUtil.kFieldLength - bluePosition.getX(), bluePosition.getY());
    }
    return bluePosition;
  }

  /**
   * Linearly interpolates flywheel RPM and hood angle for a distance between
   * two table entries. Clamps to the nearest edge if outside the table range.
   * Returns [flywheelRpm, hoodAngleDeg].
   */
  private static double[] interpolate(double[][] table, double dist) {
    // Below minimum range — use closest entry
    if (dist <= table[0][0]) {
      return new double[]{table[0][1], table[0][2]};
    }

    // Above maximum range — use farthest entry
    if (dist >= table[table.length - 1][0]) {
      return new double[]{table[table.length - 1][1], table[table.length - 1][2]};
    }

    // Find the two surrounding entries and interpolate between them
    for (int i = 0; i < table.length - 1; i++) {
      if (dist <= table[i + 1][0]) {
        // t = 0.0 means use row i exactly, t = 1.0 means use row i+1 exactly
        double t = (dist - table[i][0]) / (table[i + 1][0] - table[i][0]);
        double rpm       = table[i][1] + t * (table[i + 1][1] - table[i][1]);
        double hoodAngle = table[i][2] + t * (table[i + 1][2] - table[i][2]);
        return new double[]{rpm, hoodAngle};
      }
    }

    // Fallback — should never reach here given the checks above
    return new double[]{table[table.length - 1][1], table[table.length - 1][2]};
  }
}