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
 * This command allows the robot to aim and fire while driving.
 * It uses vector math to cancel out the robot's own movement, ensuring the ball
 * travels toward the target relative to the field, not the moving robot.
 *
 * The turret pivot is offset from the robot center. All targeting math uses the
 * turret pivot's field position (not the robot center) as the origin, so aiming
 * and distance calculations are accurate regardless of robot heading.
 */
public class ShootOnMoveCommand extends Command {
  // A simple container to hold the RPM and how long the ball stays in the air (Time of Flight)
  private record ShooterParams(double rpm, double timeOfFlight) {}

  // Field Targets: X and Y coordinates of the Hub and the "Feeding" stations
  private static final Translation2d hubPosition        = new Translation2d(8.27, 4.105);
  private static final Translation2d feedTopPosition    = new Translation2d(4.03, 6.5);
  private static final Translation2d feedBottomPosition = new Translation2d(4.03, 1.5);

  /**
   * Offset of the turret pivot from the robot's geometric center, in robot-relative
   * coordinates (X = forward, Y = left), in meters.
   *
   * Measure with a tape from your robot's center to the turret shaft and set these values.
   * This offset is rotated into field-frame each loop, so it stays correct as the robot turns.
   */
  private static final Translation2d turretOffset = new Translation2d(0.1, 0.05); // TODO: tune

  // Look-up Tables (LUT): Maps "Distance from Target" -> "Shooter Settings"
  // We use a TreeMap because it's easy to find the numbers closest to our current distance.
  private static final TreeMap<Double, ShooterParams> hubTable  = new TreeMap<>();
  private static final TreeMap<Double, ShooterParams> feedTable = new TreeMap<>();

  static {
    // Hub Table: {Distance in Meters, RPM, Seconds in Air}
    hubTable.put(1.5, new ShooterParams(2800.0, 0.42));
    hubTable.put(2.0, new ShooterParams(3100.0, 0.51));
    hubTable.put(2.5, new ShooterParams(3400.0, 0.58));
    hubTable.put(3.0, new ShooterParams(3650.0, 0.65));
    hubTable.put(3.5, new ShooterParams(3900.0, 0.71));
    hubTable.put(4.0, new ShooterParams(4100.0, 0.78));
    hubTable.put(5.0, new ShooterParams(4550.0, 0.91));

    // Feed Table: Settings used when passing the ball to the other side of the field
    feedTable.put(1.5, new ShooterParams(2200.0, 0.35));
    feedTable.put(3.0, new ShooterParams(2900.0, 0.55));
    feedTable.put(5.0, new ShooterParams(3700.0, 0.80));
    feedTable.put(8.0, new ShooterParams(4600.0, 1.18));
  }

  // Constants for tuning
  private static final double latencySeconds          = 0.15;  // Sensor + mechanical latency before ball leaves
  private static final double hubHoodAngleDeg         = 25.0;  // Fixed hood angle for Hub shots
  private static final double feedHoodAngleDeg        = 15.0;  // Fixed hood angle for Feed shots
  private static final double rpmReadyTolerance       = 150.0; // Allowed RPM error before firing
  private static final double turretReadyToleranceDeg = 1.0;   // Allowed angle error before firing
  private static final double indexerRPM              = 1500;

  private final Shooter                        shooter;
  private final ShooterHood                    hood;
  private final Turret                         turret;
  private final Indexer                        indexer;
  private final Spindexer                      spindexer;
  private final Supplier<Pose2d>               poseSupplier;
  private final Supplier<ChassisSpeeds>        speedSupplier;

  private Translation2d                        currentTarget = hubPosition;
  private TreeMap<Double, ShooterParams>       currentTable  = hubTable;

  public ShootOnMoveCommand(Shooter shooter, ShooterHood hood, Turret turret, Indexer indexer,
                            Spindexer spindexer, Supplier<Pose2d> poseSupplier,
                            Supplier<ChassisSpeeds> speedSupplier) {
    this.shooter       = shooter;
    this.hood          = hood;
    this.turret        = turret;
    this.indexer       = indexer;
    this.spindexer     = spindexer;
    this.poseSupplier  = poseSupplier;
    this.speedSupplier = speedSupplier;
    addRequirements(shooter, hood, turret, indexer, spindexer);
  }

  @Override
  public void initialize() {
    shooter.stop();
    indexer.stop();
    spindexer.stop();
  }

  @Override
  public void execute() {
    // Get where we are and how fast we are going RIGHT NOW
    Pose2d pose        = poseSupplier.get();
    ChassisSpeeds speeds = speedSupplier.get();

    // -------------------------------------------------------------------------
    // 1. PROJECT FUTURE POSITION (Latency Compensation)
    //    The sensors tell us where we were a few milliseconds ago.
    //    We predict where we WILL be when the ball actually leaves the robot.
    // -------------------------------------------------------------------------
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(pose.getRotation());

    Pose2d futurePos = new Pose2d(
        pose.getTranslation().plus(robotVel.times(latencySeconds)),
        pose.getRotation().plus(
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * latencySeconds)
        )
    );

    // -------------------------------------------------------------------------
    // 2. TURRET PIVOT POSITION (Off-Center Correction)
    //    The turret is not mounted at the robot center. We rotate the robot-relative
    //    offset into field-frame using the robot's current heading, then add it to
    //    the robot's field position. All downstream math uses this pivot point as
    //    the true origin of the shot, not the robot center.
    // -------------------------------------------------------------------------
    Translation2d turretPivotField = futurePos.getTranslation()
        .plus(turretOffset.rotateBy(futurePos.getRotation()));

    // -------------------------------------------------------------------------
    // 3. GET TARGET VECTOR & BASELINE VELOCITY
    //    Choose Hub or Feed target based on where the robot is on the field.
    //    Distance is measured from the turret pivot, not the robot center.
    // -------------------------------------------------------------------------
    selectTargetFromZone(pose);

    Translation2d toTarget = currentTarget.minus(turretPivotField);
    double distance = Math.max(toTarget.getNorm(), 0.5); // Guard against robot-inside-target edge case

    if (distance < 1.0) {
      Logger.recordOutput("SOTM/Warning", "Distance suspiciously small — check odometry");
    }

    // Look up how fast we would shoot if we were standing perfectly still.
    ShooterParams baseline             = interpolate(distance, currentTable);
    double        baselineHorizontalVel = distance / baseline.timeOfFlight;

    // -------------------------------------------------------------------------
    // 4. VECTOR SUBTRACTION (Shoot-On-Move Compensation)
    //    If you run sideways and throw a ball straight, it flies diagonally.
    //    To make it fly straight while running, aim slightly "backwards."
    //    Shot Velocity = (Goal Velocity) - (Robot Velocity)
    // -------------------------------------------------------------------------
    Translation2d targetDirection    = toTarget.div(distance);
    Translation2d targetVelocityVec  = targetDirection.times(baselineHorizontalVel);
    Translation2d shotVector         = targetVelocityVec.minus(robotVel);

    // -------------------------------------------------------------------------
    // 5. EXTRACT RESULTS & APPLY CONSTRAINTS
    //    Convert the compensated shot vector back into a turret angle.
    // -------------------------------------------------------------------------
    Rotation2d fieldAngle       = shotVector.getAngle();
    double     rawTurretAngle   = fieldAngle.minus(futurePos.getRotation()).getDegrees();

    // Keep the turret in its -180 to 180 range (no tangled wires).
    double constrainedTurret = MathUtil.inputModulus(rawTurretAngle, -180, 180);

    // Hood angle by target type; clamped to physical 0–30° limits.
    double targetHoodAngle   = (currentTable == hubTable) ? hubHoodAngleDeg : feedHoodAngleDeg;
    double constrainedHood   = MathUtil.clamp(targetHoodAngle, 0, 30);

    // -------------------------------------------------------------------------
    // 6. REVERSE LOOKUP: Required Velocity -> Effective Distance -> RPM
    //    Our vector math changed the shot speed. Find the RPM that produces
    //    the new required horizontal velocity.
    // -------------------------------------------------------------------------
    double requiredHorizontalVel = shotVector.getNorm();
    double effectiveDistance     = getDistanceForVelocity(requiredHorizontalVel, currentTable);
    double adjustedRpm           = interpolate(effectiveDistance, currentTable).rpm;

    // -------------------------------------------------------------------------
    // 7. HARDWARE OUTPUT
    // -------------------------------------------------------------------------
    shooter.setFlywheelTargetRPM(adjustedRpm);
    hood.setSetpoint(constrainedHood);
    turret.setSetpoint(constrainedTurret);
    spindexer.setSpindexerRPM(adjustedRpm * 0.25); // Slow spin to keep balls staged

    // -------------------------------------------------------------------------
    // 8. FIRING LOGIC
    //    Only feed the ball once the turret, flywheel, and hood are all ready.
    // -------------------------------------------------------------------------
    double  turretError = MathUtil.inputModulus(turret.getTurretAngle() - constrainedTurret, -180, 180);
    boolean turretReady  = Math.abs(turretError) < turretReadyToleranceDeg;
    boolean shooterReady = Math.abs(shooter.getFlywheelLeaderRPM() - adjustedRpm) < rpmReadyTolerance;

    if (turretReady && shooterReady && hood.atSetpoint()) {
      indexer.setIndexerRPM(indexerRPM);
    } else {
      indexer.stop();
    }

    // -------------------------------------------------------------------------
    // 9. LOGGING
    // -------------------------------------------------------------------------
    Logger.recordOutput("SOTM/Distance",         distance);
    Logger.recordOutput("SOTM/TurretAngle",      constrainedTurret);
    Logger.recordOutput("SOTM/HoodAngle",        constrainedHood);
    Logger.recordOutput("SOTM/AdjustedRPM",      adjustedRpm);
    Logger.recordOutput("SOTM/TurretPivotX",     turretPivotField.getX());
    Logger.recordOutput("SOTM/TurretPivotY",     turretPivotField.getY());
    Logger.recordOutput("SOTM/RequiredVel",      requiredHorizontalVel);
    Logger.recordOutput("SOTM/EffectiveDistance", effectiveDistance);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
    spindexer.stop();
    turret.stopMotor();
  }

  // ---------------------------------------------------------------------------
  // HELPERS
  // ---------------------------------------------------------------------------

  /**
   * Decides whether we should aim for the Hub or the Feeding stations based on
   * which field zone the robot is currently in.
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
   * Flips X coordinate for Red Alliance so the same target constants work for both sides.
   */
  private static Translation2d mirrorForAlliance(Translation2d bluePosition) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return new Translation2d(FieldZoneUtil.kFieldLength - bluePosition.getX(), bluePosition.getY());
    }
    return bluePosition;
  }

  /**
   * Interpolates shooter parameters for a distance that falls between two table entries.
   * Returns the nearest boundary value if distance is outside the table range.
   */
  private static ShooterParams interpolate(double distance, TreeMap<Double, ShooterParams> table) {
    Map.Entry<Double, ShooterParams> lo = table.floorEntry(distance);
    Map.Entry<Double, ShooterParams> hi = table.ceilingEntry(distance);

    if (lo == null) return hi.getValue(); // Closer than table minimum
    if (hi == null) return lo.getValue(); // Farther than table maximum

    // Exact match — avoid division by zero
    if (lo.getKey().equals(hi.getKey())) return lo.getValue();

    double t = (distance - lo.getKey()) / (hi.getKey() - lo.getKey());
    return new ShooterParams(
        lo.getValue().rpm + t * (hi.getValue().rpm - lo.getValue().rpm),
        lo.getValue().timeOfFlight + t * (hi.getValue().timeOfFlight - lo.getValue().timeOfFlight)
    );
  }

  /**
   * Reverse lookup: given a required horizontal ball velocity, finds the effective
   * distance in the table that would produce that velocity (vel = dist / tof).
   * Interpolates between adjacent entries; clamps to max distance if velocity exceeds table.
   */
  private static double getDistanceForVelocity(double targetVel, TreeMap<Double, ShooterParams> table) {
    double prevVel  = -1;
    double prevDist = -1;

    for (var entry : table.entrySet()) {
      double dist = entry.getKey();
      double vel  = dist / entry.getValue().timeOfFlight; // v = d / t

      if (prevVel >= 0 && targetVel <= vel) {
        double t = (targetVel - prevVel) / (vel - prevVel);
        return prevDist + t * (dist - prevDist);
      }
      prevVel  = vel;
      prevDist = dist;
    }
    return table.lastKey(); // Velocity exceeds table — use max distance
  }
}