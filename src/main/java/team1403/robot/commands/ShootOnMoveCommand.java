package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import team1403.robot.subsystems.*;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Side;
import team1403.robot.util.FieldZoneUtil.Zone;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;

/**
 * ShootOnMoveCommand
 * Allows the robot to aim and fire while driving.
 * Uses vector math to cancel out robot movement so the ball travels
 * toward the target relative to the field, not the moving robot.
 *
 * The turret pivot is offset from the robot center. All targeting math
 * uses the turret pivot's field position as the origin so aiming and
 * distance calculations stay accurate regardless of robot heading.
 */
public class ShootOnMoveCommand extends Command {

  // Holds the RPM and estimated time the ball spends in the air for a given distance.
  private record ShooterParams(double rpm, double timeOfFlight) {}

  // Fixed field positions for the Hub and the two Feeding stations (Blue alliance coords).
  private static final Translation2d hubPosition        = new Translation2d(8.27, 4.105);
  private static final Translation2d feedTopPosition    = new Translation2d(4.03, 6.5);
  private static final Translation2d feedBottomPosition = new Translation2d(4.03, 1.5);

  /**
   * How far the turret pivot is from the robot's center, in robot-relative meters
   * (X = forward, Y = left). This offset is rotated into field-frame every loop
   * so it stays accurate as the robot turns.
   * TODO: Measure from the robot and update these values before competition.
   */
  private static final Translation2d turretOffset = new Translation2d(0.1, 0.05);

  // Distance-to-shooter-settings lookup tables.
  // TreeMap lets us quickly find the two closest entries and interpolate between them.
  private static final TreeMap<Double, ShooterParams> hubTable  = new TreeMap<>();
  private static final TreeMap<Double, ShooterParams> feedTable = new TreeMap<>();

  static {
    // Hub shots: distance (meters) -> RPM and time-of-flight (seconds)
    hubTable.put(1.5, new ShooterParams(2800.0, 0.42));
    hubTable.put(2.0, new ShooterParams(3100.0, 0.51));
    hubTable.put(2.5, new ShooterParams(3400.0, 0.58));
    hubTable.put(3.0, new ShooterParams(3650.0, 0.65));
    hubTable.put(3.5, new ShooterParams(3900.0, 0.71));
    hubTable.put(4.0, new ShooterParams(4100.0, 0.78));
    hubTable.put(5.0, new ShooterParams(4550.0, 0.91));

    // Feed shots: used when passing the ball across the field to an ally
    feedTable.put(1.5, new ShooterParams(2200.0, 0.35));
    feedTable.put(3.0, new ShooterParams(2900.0, 0.55));
    feedTable.put(5.0, new ShooterParams(3700.0, 0.80));
    feedTable.put(8.0, new ShooterParams(4600.0, 1.18));
  }

  // Tuning constants — adjust these based on robot behavior.
  private static final double latencySeconds          = 0.15;  // Time between sensor read and ball leaving robot
  private static final double hubHoodAngleDeg         = 25.0;  // Hood angle for Hub shots
  private static final double feedHoodAngleDeg        = 15.0;  // Hood angle for Feed shots
  private static final double rpmReadyTolerance       = 150.0; // Max RPM error allowed before firing
  private static final double turretReadyToleranceDeg = 1.0;   // Max turret angle error allowed before firing
  private static final double indexerRPM              = 1500.0;// Speed to run the indexer when feeding a ball
  private static final double spindexerRPMRatio       = 0.25;  // Spindexer runs at this fraction of flywheel RPM

  private final Shooter              shooter;
  private final ShooterHood          hood;
  private final Turret               turret;
  private final Indexer              indexer;
  private final Spindexer            spindexer;
  private final Supplier<Pose2d>     poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;

  private Translation2d                  currentTarget = hubPosition;
  private TreeMap<Double, ShooterParams> currentTable  = hubTable;

  // Tracks whether we are currently feeding a ball. Prevents redundant stop() calls every loop.
  private boolean isFeeding = false;

  public ShootOnMoveCommand(Shooter shooter, ShooterHood hood, Turret turret, Indexer indexer,
                            Spindexer spindexer, Supplier<Pose2d> poseSupplier,
                            Supplier<ChassisSpeeds> speedSupplier) {
    this.shooter      = shooter;
    this.hood         = hood;
    this.turret       = turret;
    this.indexer      = indexer;
    this.spindexer    = spindexer;
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    addRequirements(shooter, hood, turret, indexer, spindexer);
  }

  @Override
  public void initialize() {
    // Only stop the feeder motors on start. The flywheel is left alone intentionally —
    // if it was already spinning from a previous command we keep that speed and avoid
    // a full re-spin-up delay.
    indexer.stop();
    spindexer.stop();
    isFeeding = false;
  }

  @Override
  public void execute() {
    // Grab current position and velocity from the swerve drive.
    Pose2d        pose   = poseSupplier.get();
    ChassisSpeeds speeds = speedSupplier.get();

    // -------------------------------------------------------------------------
    // 1. PROJECT FUTURE POSITION (Latency Compensation)
    //    Sensors report where the robot WAS a few milliseconds ago. We project
    //    forward in time so our aim is based on where the robot WILL be when
    //    the ball actually leaves the shooter.
    // -------------------------------------------------------------------------
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(pose.getRotation()); // Convert from robot-relative to field-relative velocity

    Pose2d futurePos = new Pose2d(
        pose.getTranslation().plus(robotVel.times(latencySeconds)),
        pose.getRotation().plus(
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * latencySeconds)
        )
    );

    // -------------------------------------------------------------------------
    // 2. TURRET PIVOT POSITION (Off-Center Correction)
    //    The turret is not at the robot center. We rotate the robot-relative
    //    turret offset into field coordinates using the robot's heading, then
    //    add it to the robot's future position. All shot math below uses this
    //    pivot point — NOT the robot center — for accurate aiming.
    // -------------------------------------------------------------------------
    Translation2d turretPivotField = futurePos.getTranslation()
        .plus(turretOffset.rotateBy(futurePos.getRotation()));

    // -------------------------------------------------------------------------
    // 3. SELECT TARGET & MEASURE DISTANCE
    //    Pick Hub or Feed target based on which zone the robot is in.
    //    Measure distance from the turret pivot for maximum accuracy.
    // -------------------------------------------------------------------------
    selectTargetFromZone(pose);

    Translation2d toTarget = currentTarget.minus(turretPivotField);
    double distance = toTarget.getNorm();

    // If distance is unrealistically small, something is wrong with odometry.
    // Clamp to 0.5m so we don't divide by zero or produce garbage outputs,
    // and skip the rest of this loop to avoid acting on bad data.
    if (distance < 1.0) {
      Logger.recordOutput("SOTM/Warning", "Distance < 1.0m — skipping loop, check odometry");
      distance = Math.max(distance, 0.5);
      return;
    }

    // Look up how fast the ball needs to travel if we were standing still.
    ShooterParams baseline            = interpolate(distance, currentTable);
    double        baselineHorizontalVel = distance / baseline.timeOfFlight;

    // -------------------------------------------------------------------------
    // 4. VECTOR SUBTRACTION (Shoot-On-Move Compensation)
    //    A ball shot from a moving robot inherits the robot's velocity.
    //    To make the ball travel straight to the target, we aim slightly
    //    "against" the direction of travel so the two vectors cancel out.
    //    Formula: shotVector = targetVelocityVector - robotVelocity
    // -------------------------------------------------------------------------
    Translation2d targetDirection   = toTarget.div(distance);
    Translation2d targetVelocityVec = targetDirection.times(baselineHorizontalVel);
    Translation2d shotVector        = targetVelocityVec.minus(robotVel);

    // -------------------------------------------------------------------------
    // 5. CONVERT SHOT VECTOR TO HARDWARE TARGETS
    //    Turn the compensated shot vector into a turret angle and hood angle.
    // -------------------------------------------------------------------------
    Rotation2d fieldAngle     = shotVector.getAngle();
    double     rawTurretAngle = fieldAngle.minus(futurePos.getRotation()).getDegrees();

    // Wrap turret angle to -180..180 so we never try to spin past the wire stop.
    double constrainedTurret = MathUtil.inputModulus(rawTurretAngle, -180, 180);

    // Hood angle is fixed per target type. Clamped to physical limits of 0–30°.
    double targetHoodAngle = (currentTable == hubTable) ? hubHoodAngleDeg : feedHoodAngleDeg;
    double constrainedHood = MathUtil.clamp(targetHoodAngle, 0, 30);

    // -------------------------------------------------------------------------
    // 6. REVERSE LOOKUP: Compensated Velocity -> Effective Distance -> RPM
    //    The vector compensation changed the required shot speed. We find what
    //    distance in the table produces that speed and look up the matching RPM.
    // -------------------------------------------------------------------------
    double requiredHorizontalVel = shotVector.getNorm();
    double effectiveDistance     = getDistanceForVelocity(requiredHorizontalVel, currentTable);
    double adjustedRpm           = interpolate(effectiveDistance, currentTable).rpm;

    // -------------------------------------------------------------------------
    // 7. HARDWARE OUTPUT
    //    Send final targets to each mechanism. Spindexer is set once here at a
    //    fixed ratio to keep balls staged — it does NOT reset to zero each loop.
    // -------------------------------------------------------------------------
    shooter.setFlywheelTargetRPM(adjustedRpm);
    hood.setSetpoint(constrainedHood);
    turret.setSetpoint(constrainedTurret);
    spindexer.setSpindexerRPM(adjustedRpm * spindexerRPMRatio);

    // -------------------------------------------------------------------------
    // 8. FIRING LOGIC
    //    Gate the indexer behind all three readiness checks. We use a state flag
    //    (isFeeding) so we only call setIndexerRPM / stop once on each transition
    //    instead of hammering the motor controller every 20ms.
    // -------------------------------------------------------------------------
    double  turretError  = MathUtil.inputModulus(turret.getTurretAngle() - constrainedTurret, -180, 180);
    boolean turretReady  = Math.abs(turretError) < turretReadyToleranceDeg;
    boolean shooterReady = Math.abs(shooter.getFlywheelLeaderRPM() - adjustedRpm) < rpmReadyTolerance;
    boolean hoodReady    = hood.atSetpoint(); // ShooterHood.atSetpoint() must use a tight tolerance

    boolean readyToFire = turretReady && shooterReady && hoodReady;

    if (readyToFire && !isFeeding) {
      indexer.setIndexerRPM(indexerRPM);
      isFeeding = true;
    } else if (!readyToFire && isFeeding) {
      indexer.stop();
      isFeeding = false;
    }

    // -------------------------------------------------------------------------
    // 9. LOGGING
    // -------------------------------------------------------------------------
    Logger.recordOutput("SOTM/Distance",          distance);
    Logger.recordOutput("SOTM/TurretAngle",       constrainedTurret);
    Logger.recordOutput("SOTM/HoodAngle",         constrainedHood);
    Logger.recordOutput("SOTM/AdjustedRPM",       adjustedRpm);
    Logger.recordOutput("SOTM/TurretPivotX",      turretPivotField.getX());
    Logger.recordOutput("SOTM/TurretPivotY",      turretPivotField.getY());
    Logger.recordOutput("SOTM/RequiredVel",       requiredHorizontalVel);
    Logger.recordOutput("SOTM/EffectiveDistance", effectiveDistance);
    Logger.recordOutput("SOTM/TurretReady",       turretReady);
    Logger.recordOutput("SOTM/ShooterReady",      shooterReady);
    Logger.recordOutput("SOTM/HoodReady",         hoodReady);
    Logger.recordOutput("SOTM/IsFeeding",         isFeeding);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop everything cleanly when the command ends (button released or interrupted).
    shooter.stop();
    indexer.stop();
    spindexer.stop();
    turret.stopMotor();
    isFeeding = false;
  }

  // ---------------------------------------------------------------------------
  // HELPERS
  // ---------------------------------------------------------------------------

  /**
   * Picks Hub or Feed as the target based on which field zone the robot occupies.
   * Zone.MY_ALLIANCE = aim at the Hub. Any other zone = aim at the nearest Feed station.
   */
  private void selectTargetFromZone(Pose2d pose) {
    Zone zone = FieldZoneUtil.getZone(pose);
    Side side = FieldZoneUtil.getSide(pose);

    if (zone == Zone.MY_ALLIANCE) {
      currentTarget = mirrorForAlliance(hubPosition);
      currentTable  = hubTable;
    } else {
      Translation2d rawFeedPos = (side == Side.TOP) ? feedTopPosition : feedBottomPosition;
      currentTarget = mirrorForAlliance(rawFeedPos);
      currentTable  = feedTable;
    }
  }

  /**
   * Mirrors a Blue-alliance field position to the Red side by flipping its X coordinate.
   * This lets us define all targets once in Blue coordinates and use them for both alliances.
   */
  private static Translation2d mirrorForAlliance(Translation2d bluePosition) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return new Translation2d(FieldZoneUtil.kFieldLength - bluePosition.getX(), bluePosition.getY());
    }
    return bluePosition;
  }

  /**
   * Linearly interpolates shooter parameters (RPM, time-of-flight) for a distance
   * between two table entries. Returns the nearest edge value if distance is out of range.
   */
  private static ShooterParams interpolate(double distance, TreeMap<Double, ShooterParams> table) {
    Map.Entry<Double, ShooterParams> lo = table.floorEntry(distance);
    Map.Entry<Double, ShooterParams> hi = table.ceilingEntry(distance);

    if (lo == null) return hi.getValue(); // Closer than the closest table entry
    if (hi == null) return lo.getValue(); // Farther than the farthest table entry
    if (lo.getKey().equals(hi.getKey())) return lo.getValue(); // Exact match, avoid divide-by-zero

    double t = (distance - lo.getKey()) / (hi.getKey() - lo.getKey()); // 0.0 = lo, 1.0 = hi
    return new ShooterParams(
        lo.getValue().rpm            + t * (hi.getValue().rpm            - lo.getValue().rpm),
        lo.getValue().timeOfFlight   + t * (hi.getValue().timeOfFlight   - lo.getValue().timeOfFlight)
    );
  }

  /**
   * Reverse lookup: given a required horizontal ball speed, find the table distance
   * that produces that speed (speed = distance / time-of-flight). Interpolates between
   * entries. If the required speed exceeds the whole table, returns the max distance.
   */
  private static double getDistanceForVelocity(double targetVel, TreeMap<Double, ShooterParams> table) {
    double prevVel  = -1;
    double prevDist = -1;

    for (var entry : table.entrySet()) {
      double dist = entry.getKey();
      double vel  = dist / entry.getValue().timeOfFlight;

      if (prevVel >= 0 && targetVel <= vel) {
        double t = (targetVel - prevVel) / (vel - prevVel);
        return prevDist + t * (dist - prevDist);
      }
      prevVel  = vel;
      prevDist = dist;
    }
    return table.lastKey(); // Required speed is beyond the table max — clamp to farthest entry
  }
}