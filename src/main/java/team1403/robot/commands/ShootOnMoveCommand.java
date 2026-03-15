package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import team1403.robot.Constants;
import team1403.robot.subsystems.*;
import team1403.robot.util.Blackbox;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;

/**
 * This command allows the robot to score while it is physically moving across the field.
 * Without this compensation, a ball shot from a moving robot would drift off-target because
 * it inherits the robot's velocity. This command corrects for that by doing vector math:
 * we figure out which direction and speed the ball needs to leave the robot so that after
 * adding the robot's own movement, the ball still arrives at the target.
 */
public class ShootOnMoveCommand extends Command {
  // Combines the two shooter settings we need for any given shot distance.
  private record ShooterParams(double rpm, double timeOfFlight) {}

  // Lookup tables mapping shot distance (meters) to flywheel RPM and time-of-flight.
  // Two separate tables exist because hub shots and feed shots have different trajectories.
  // TreeMap is used so we can efficiently find and interpolate between the two nearest entries.
  private static final TreeMap<Double, ShooterParams> hubTable  = new TreeMap<>();
  private static final TreeMap<Double, ShooterParams> feedTable = new TreeMap<>();

  static {
    // Hub shot table — scoring directly into the hub
    hubTable.put(0.5, new ShooterParams(2200.0, 0.22));
    hubTable.put(1.0, new ShooterParams(2500.0, 0.33));
    hubTable.put(1.5, new ShooterParams(2800.0, 0.42));
    hubTable.put(2.0, new ShooterParams(3100.0, 0.51));
    hubTable.put(2.5, new ShooterParams(3400.0, 0.58));
    hubTable.put(3.0, new ShooterParams(3650.0, 0.65));
    hubTable.put(3.5, new ShooterParams(3900.0, 0.71));
    hubTable.put(4.0, new ShooterParams(4100.0, 0.78));
    hubTable.put(5.0, new ShooterParams(4550.0, 0.91));

    // Feed shot table — passing the ball to an ally across the field
    feedTable.put(1.5, new ShooterParams(2200.0, 0.35));
    feedTable.put(3.0, new ShooterParams(2900.0, 0.55));
    feedTable.put(5.0, new ShooterParams(3700.0, 0.80));
    feedTable.put(8.0, new ShooterParams(4600.0, 1.18));
  }

  // How far in the future (seconds) we project the robot's position to account for
  // the delay between sensor readings and the ball actually leaving the robot.
  private static final double kLatencySeconds = 0.20;

  // Fixed hood angles per shot type. 
  private static final double kHubHoodAngleDeg = 10.0;
  private static final double kFeedHoodAngleDeg = 25.0;

  // The flywheel must be within this much RPM 
  private static final double kRpmReadyTolerance = 90.0;

  // The indexer and spindexer always run at these fixed speeds when feeding a ball.
  // These do not change based on distance or flywheel RPM.
  private static final double kIndexerRPM = 1500.0;
  private static final double kSpindexerRPM = 375.0;

  private final Shooter shooter;
  private final ShooterHood hood;
  private final Turret turret;
  private final Indexer indexer;
  private final Spindexer spindexer;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;

  // Tracks which lookup table is active this loop so RPM interpolation uses the right data.
  private TreeMap<Double, ShooterParams> currentTable = hubTable;

  // True when the indexer is currently running. Used to avoid calling motor methods
  // every single loop — we only call them once on each state transition.
  private boolean isFeeding = false;

  public ShootOnMoveCommand(Shooter shooter, ShooterHood hood, Turret turret, Indexer indexer,
                            Spindexer spindexer, Supplier<Pose2d> poseSupplier,
                            Supplier<ChassisSpeeds> speedSupplier) {
    this.shooter = shooter;
    this.hood = hood;
    this.turret = turret;
    this.indexer = indexer;
    this.spindexer = spindexer;
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    addRequirements(shooter, hood, indexer, spindexer);
  }

  // Called once when the command starts. We stop the feeder motors in case they were
  // left running from a previous command. The flywheel is intentionally left alone so
  // we do not waste time waiting for it to re-spin if it was already at speed.
  @Override
  public void initialize() {
    indexer.stop();
    spindexer.stop();
    isFeeding = false;
  }

  @Override
  public void execute() {
    Pose2d        pose   = poseSupplier.get();
    ChassisSpeeds speeds = speedSupplier.get();

    // STEP 1 — LATENCY COMPENSATION
    // Odometry tells us where the robot was a short time ago, not where it is right now.
    // We project the robot's position forward by kLatencySeconds so our calculations
    // are based on where the robot will actually be when the ball leaves the shooter.
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(pose.getRotation());

    Pose2d futurePos = new Pose2d(
        pose.getTranslation().plus(robotVel.times(kLatencySeconds)),
        pose.getRotation().plus(
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * kLatencySeconds)
        )
    );

    // STEP 2 — TARGET SELECTION AND DISTANCE MEASUREMENT
    // Blackbox.getActiveTarget() picks the hub or the correct feed station based on
    // which zone the robot is currently in, and mirrors it to the correct alliance side.
    // turret.getDistanceToTarget() measures from the turret pivot (not the robot center)
    // using the offset already defined in the Turret subsystem.
    Translation2d currentTarget = Blackbox.getActiveTarget(futurePos);

    if (Blackbox.getActiveTarget(pose) == Constants.ScoringLocation.kHubPosition) {
      currentTable = hubTable;
    } else {
      currentTable = feedTable;
    }

    double distance = turret.getDistanceToTarget(futurePos);
    Translation2d toTarget = currentTarget.minus(futurePos.getTranslation()
        .plus(Constants.Turret.kTurretOffset.rotateBy(futurePos.getRotation())));

    // If the distance is suspiciously small, odometry is likely producing bad data.
    // We log a warning and skip this loop to avoid commanding garbage values.
    if (distance < 0.1) {
      Logger.recordOutput("SOTM/Warning", "Distance < 0.1m — skipping loop, check odometry");
      return;
    }

    // Look up what RPM and time-of-flight the robot would need if it were standing still.
    ShooterParams baseline = interpolate(distance, currentTable);
    double baselineHorizVel = distance / baseline.timeOfFlight;

    // STEP 3 — SHOOT-ON-MOVE VECTOR COMPENSATION
    // A ball launched from a moving robot inherits the robot's velocity. To make the
    // ball travel straight to the target, we subtract the robot's velocity from the
    // required ball velocity. The result is the direction and speed the ball must leave
    // the robot so that the two velocities add up to the correct field-relative path.
    Translation2d targetDirection   = toTarget.div(distance);
    Translation2d targetVelocityVec = targetDirection.times(baselineHorizVel);
    Translation2d shotVector        = targetVelocityVec.minus(robotVel);

    // STEP 4 — HOOD ANGLE
    // The hood angle is fixed per shot type and does not vary with distance.
    // It is clamped to the physical travel limits of the hood mechanism (0–30 degrees).
    double targetHoodAngle;
    if (currentTable == hubTable) {
      targetHoodAngle = kHubHoodAngleDeg;
    } else {
      targetHoodAngle = kFeedHoodAngleDeg;
    }
    double constrainedHood = MathUtil.clamp(targetHoodAngle, 0, 30);

    // STEP 5 — CONVERT COMPENSATED SHOT VECTOR BACK TO RPM
    // The vector compensation changed the required ball speed. We reverse-look up which
    // distance in the table produces that speed, then interpolate to get the matching RPM.
    double requiredHorizVel  = shotVector.getNorm();
    double effectiveDistance = getDistanceForVelocity(requiredHorizVel, currentTable);
    double adjustedRpm = interpolate(effectiveDistance, currentTable).rpm;

    // STEP 6 — SEND TARGETS TO HARDWARE
    // Only the flywheel RPM and hood angle change shot to shot. The spindexer and
    // indexer run at constant speeds and are only gated by the readiness check below.
    shooter.setFlywheelTargetRPM(adjustedRpm);
    hood.setSetpoint(constrainedHood);

    // STEP 7 — FIRING GATE
    // We only feed a ball once both the flywheel and hood are at their targets.
    // The isFeeding flag means we only call motor methods on the transition in or out,
    // not every 20ms loop, which avoids unnecessary CAN traffic.
    boolean shooterReady = Math.abs(shooter.getFlywheelLeaderRPM() - adjustedRpm) < kRpmReadyTolerance;
    boolean hoodReady = hood.atSetpoint();
    boolean readyToFire = shooterReady && hoodReady;

    if (readyToFire && !isFeeding) {
      indexer.setIndexerRPM(kIndexerRPM);
      spindexer.setSpindexerRPM(kSpindexerRPM);
      isFeeding = true;
    } else if (!readyToFire && isFeeding) {
      indexer.stop();
      spindexer.stop();
      isFeeding = false;
    }

  
    Logger.recordOutput("SOTM/Distance", distance);
    Logger.recordOutput("SOTM/HoodAngle", constrainedHood);
    Logger.recordOutput("SOTM/AdjustedRPM", adjustedRpm);
    Logger.recordOutput("SOTM/RequiredVel", requiredHorizVel);
    Logger.recordOutput("SOTM/EffectiveDistance", effectiveDistance);
    Logger.recordOutput("SOTM/ShooterReady", shooterReady);
    Logger.recordOutput("SOTM/HoodReady", hoodReady);
    Logger.recordOutput("SOTM/IsFeeding", isFeeding);
  }


  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
    spindexer.stop();
    isFeeding = false;
  }


  /**
   * Linearly interpolates between the two nearest entries in a lookup table.
   * For example, if the distance is 2.3m and the table has entries at 2.0m and 2.5m,
   * this returns values 60% of the way between those two entries.
   * If the distance is outside the table range, the nearest edge entry is returned as-is.
   */
  private static ShooterParams interpolate(double distance, TreeMap<Double, ShooterParams> table) {
    Map.Entry<Double, ShooterParams> lo = table.floorEntry(distance);
    Map.Entry<Double, ShooterParams> hi = table.ceilingEntry(distance);

    if (lo == null) return hi.getValue();
    if (hi == null) return lo.getValue();
    if (lo.getKey().equals(hi.getKey())) return lo.getValue();

    double t = (distance - lo.getKey()) / (hi.getKey() - lo.getKey());
    return new ShooterParams(
        lo.getValue().rpm          + t * (hi.getValue().rpm          - lo.getValue().rpm),
        lo.getValue().timeOfFlight + t * (hi.getValue().timeOfFlight - lo.getValue().timeOfFlight)
    );
  }

  /**
   * Reverse lookup: given a required ball speed (meters/second), finds the corresponding
   * distance in the table that produces that speed using speed = distance / timeOfFlight.
   * Interpolates between entries just like the forward lookup above.
   * If the required speed is faster than anything in the table, returns the max table distance.
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
    return table.lastKey();
  }
}