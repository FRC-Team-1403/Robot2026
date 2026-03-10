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
 */
public class ShootOnMoveCommand extends Command {
  // A simple container to hold the RPM and how long the ball stays in the air (Time of Flight)
  private record ShooterParams(double rpm, double timeOfFlight) {}

  // Field Targets: X and Y coordinates of the Hub and the "Feeding" stations
  private static final Translation2d hubPosition = new Translation2d(8.27, 4.105);
  private static final Translation2d feedTopPosition = new Translation2d(4.03, 6.5);
  private static final Translation2d feedBottomPosition = new Translation2d(4.03, 1.5);

  // Look-up Tables (LUT): Maps "Distance from Target" -> "Shooter Settings"
  // We use a TreeMap because it's easy to find the numbers closest to our current distance.
  private static final TreeMap<Double, ShooterParams> hubTable = new TreeMap<>();
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
  private static final double latencySeconds = 0.15; // Time it takes for sensors to process + ball to leave
  private static final double hubHoodAngleDeg = 25.0; // Fixed hood angle for Hub shots
  private static final double feedHoodAngleDeg = 15.0; // Fixed hood angle for Feed shots
  private static final double rpmReadyTolerance = 150.0; // Allowed RPM error before firing
  private static final double turretReadyToleranceDeg = 1.0; // Allowed angle error before firing

  private final Shooter shooter;
  private final ShooterHood hood;
  private final Turret turret;
  private final Indexer indexer;
  private final Spindexer spindexer;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;

  private Translation2d currentTarget = hubPosition;
  private TreeMap<Double, ShooterParams> currentTable = hubTable;

  public ShootOnMoveCommand(Shooter shooter, ShooterHood hood, Turret turret, Indexer indexer, 
                            Spindexer spindexer, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
    this.shooter = shooter;
    this.hood = hood;
    this.turret = turret;
    this.indexer = indexer;
    this.spindexer = spindexer;
    this.poseSupplier = poseSupplier;
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
    Pose2d pose = poseSupplier.get();
    ChassisSpeeds speeds = speedSupplier.get();

    // 1. PROJECT FUTURE POSITION (Latency Compensation)
    // The sensors tell us where we were a few milliseconds ago. 
    // We predict where we WILL be when the ball actually leaves the robot.
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Translation2d futurePos = pose.getTranslation().plus(robotVel.times(latencySeconds));

    // 2. GET TARGET VECTOR & BASELINE VELOCITY
    // Choose if we are shooting at the Hub or Feeding based on where the robot is standing.
    selectTargetFromZone(pose);
    
    // Calculate the distance from our future self to the target.
    Translation2d toTarget = currentTarget.minus(futurePos);
    double distance = Math.max(toTarget.getNorm(), 0.5); // Ensure distance is at least 0.5m to avoid math errors
    
    // Look up how fast we would shoot if we were standing perfectly still.
    ShooterParams baseline = interpolate(distance, currentTable);
    // Horizontal velocity is "Distance / Time". If we want to travel 5m in 1s, we need 5m/s.
    double baselineHorizontalVel = distance / baseline.timeOfFlight;

    // 3. VECTOR SUBTRACTION 
    // If you run sideways and throw a ball straight, it flies diagonally.
    // To make it fly straight while running, you must aim slightly "backwards."
    // Shot Velocity = (Goal Velocity) - (Robot Velocity)
    Translation2d targetDirection = toTarget.div(distance); // The "pointing" direction to the goal
    Translation2d targetVelocityVec = targetDirection.times(baselineHorizontalVel); // The "ideal" arrow
    Translation2d shotVector = targetVelocityVec.minus(robotVel); // The "compensated" arrow

    // 4. EXTRACT RESULTS & APPLY CONSTRAINTS
    // Turn the "compensated" arrow back into an angle for the turret.
    Rotation2d fieldAngle = shotVector.getAngle();
    double rawTurretAngle = fieldAngle.minus(pose.getRotation()).getDegrees();
    
    // Ensure the turret stays in its -180 to 180 range (no tangled wires).
    double constrainedTurret = MathUtil.inputModulus(rawTurretAngle, -180, 180);

    // Pick the hood angle based on our target.
    double targetHoodAngle;
    if (currentTable == hubTable) {
      targetHoodAngle = hubHoodAngleDeg;
    } else {
      targetHoodAngle = feedHoodAngleDeg;
    }
    // Safety check: Keep the hood within its physical 0-30 degree limits.
    double constrainedHood = MathUtil.clamp(targetHoodAngle, 0, 30);

    // 5. REVERSE LOOKUP: Velocity -> RPM
    // Our vector math gave us a "Required Speed." We check our table to see 
    // what RPM and Distance normally create that speed.
    double requiredHorizontalVel = shotVector.getNorm();
    double effectiveDistance = getDistanceForVelocity(requiredHorizontalVel, currentTable);
    double adjustedRpm = interpolate(effectiveDistance, currentTable).rpm;

    // 6. HARDWARE OUTPUT
    // Tell the motors what to do!
    shooter.setFlywheelTargetRPM(adjustedRpm);
    hood.setSetpoint(constrainedHood);
    turret.setSetpoint(constrainedTurret);
    spindexer.setSpindexerRPM(adjustedRpm * 0.25); // Slow spin to keep balls ready

    // 7. FIRING LOGIC
    // Only fire if the Turret is aimed, the Flywheel is up to speed, and the Hood is set.
    boolean turretReady = Math.abs(turret.getTurretAngle() - constrainedTurret) < turretReadyToleranceDeg;
    boolean shooterReady = Math.abs(shooter.getFlywheelLeaderRPM() - adjustedRpm) < rpmReadyTolerance;
    
    if (turretReady && shooterReady && hood.atSetpoint()) {
      indexer.setIndexerRPM(adjustedRpm * 0.5); // Feed ball into shooter
    } else {
      indexer.stop();
    }

    // 8. LOGGING (Sending data to the dashboard for debugging)
    Logger.recordOutput("SOTM/Distance", distance);
    Logger.recordOutput("SOTM/TurretAngle", constrainedTurret);
    Logger.recordOutput("SOTM/HoodAngle", constrainedHood);
    Logger.recordOutput("SOTM/AdjustedRPM", adjustedRpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
    spindexer.stop();
    turret.stopMotor();
  }

  /**
   * Decides whether we should aim for the Hub or the Feeding stations.
   */
  private void selectTargetFromZone(Pose2d pose) {
    Zone zone = FieldZoneUtil.getZone(pose);
    Side side = FieldZoneUtil.getSide(pose);

    if (zone == Zone.MY_ALLIANCE) {
      currentTarget = mirrorForAlliance(hubPosition);
      currentTable = hubTable;
    } else {
      Translation2d rawFeedPos;
      if (side == Side.TOP) {
        rawFeedPos = feedTopPosition;
      } else {
        rawFeedPos = feedBottomPosition;
      }
      currentTarget = mirrorForAlliance(rawFeedPos);
      currentTable = feedTable;
    }
  }

  /**
   * Flips coordinates if we are on the Red Alliance, so the code works for both sides.
   */
  private static Translation2d mirrorForAlliance(Translation2d bluePosition) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return new Translation2d(FieldZoneUtil.kFieldLength - bluePosition.getX(), bluePosition.getY());
    } else {
      return bluePosition;
    }
  }

  /**
   * Interpolation: If the table has values for 2m and 3m, but we are at 2.5m, 
   * this finds the perfect "middle" value.
   */
  private static ShooterParams interpolate(double distance, TreeMap<Double, ShooterParams> table) {
    Map.Entry<Double, ShooterParams> lo = table.floorEntry(distance);
    Map.Entry<Double, ShooterParams> hi = table.ceilingEntry(distance);
    
    if (lo == null) { return hi.getValue(); } // Too close, use smallest value
    if (hi == null) { return lo.getValue(); } // Too far, use largest value
    
    // Calculate how far we are between "Low" and "High" (0.0 to 1.0)
    double t = (distance - lo.getKey()) / (hi.getKey() - lo.getKey());
    
    // Blend the values based on that percentage
    return new ShooterParams(
      lo.getValue().rpm + t * (hi.getValue().rpm - lo.getValue().rpm),
      lo.getValue().timeOfFlight + t * (hi.getValue().timeOfFlight - lo.getValue().timeOfFlight)
    );
  }

  /**
   * Reverse Lookup: Finds which "Distance" in our table corresponds to a specific ball velocity.
   */
  private static double getDistanceForVelocity(double targetVel, TreeMap<Double, ShooterParams> table) {
    double prevVel = -1;
    double prevDist = -1;
    
    for (var entry : table.entrySet()) {
      double dist = entry.getKey();
      double vel = dist / entry.getValue().timeOfFlight; // Velocity = D / T
      
      // If the target speed is between the previous point and this point, interpolate.
      if (prevVel >= 0 && targetVel <= vel) {
        double t = (targetVel - prevVel) / (vel - prevVel);
        return prevDist + t * (dist - prevDist);
      }
      prevVel = vel;
      prevDist = dist;
    }
    return table.lastKey(); // If too fast, just use the max distance
  }
}