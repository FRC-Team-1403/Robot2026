package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.Blackbox;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;

/**

 * This command runs the entire time the robot is enabled. It continuously
 * points the turret at the correct target (Hub or Feed station) regardless
 * of what the robot is doing — driving, shooting stationary, or shooting on the move.
 */
public class TurretTrackingCommand extends Command {

  // Bundles RPM and time-of-flight for a given distance — same record as SOM.
  private record ShooterParams(double rpm, double timeOfFlight) {}

  // How far in the future (seconds) we project the robot's position.
  // Accounts for the delay between sensor readings and the ball leaving the robot.
  private static final double kLatencySeconds = 0.20;

  // Same lookup tables as ShootOnMoveCommand. Time-of-flight is the only value
  // we use here — it tells us how long the ball is in the air at each distance
  // so the velocity lead compensation scales correctly with range.
  private static final TreeMap<Double, ShooterParams> hubTable  = new TreeMap<>();
  private static final TreeMap<Double, ShooterParams> feedTable = new TreeMap<>();

  static {
    hubTable.put(0.5, new ShooterParams(2200.0, 0.22));
    hubTable.put(1.0, new ShooterParams(2500.0, 0.33));
    hubTable.put(1.5, new ShooterParams(2800.0, 0.42));
    hubTable.put(2.0, new ShooterParams(3100.0, 0.51));
    hubTable.put(2.5, new ShooterParams(3400.0, 0.58));
    hubTable.put(3.0, new ShooterParams(3650.0, 0.65));
    hubTable.put(3.5, new ShooterParams(3900.0, 0.71));
    hubTable.put(4.0, new ShooterParams(4100.0, 0.78));
    hubTable.put(5.0, new ShooterParams(4550.0, 0.91));

    feedTable.put(1.5, new ShooterParams(2200.0, 0.35));
    feedTable.put(3.0, new ShooterParams(2900.0, 0.55));
    feedTable.put(5.0, new ShooterParams(3700.0, 0.80));
    feedTable.put(8.0, new ShooterParams(4600.0, 1.18));
  }

  private final Turret                  m_turret;
  private final Supplier<Pose2d>        m_poseSupplier;
  private final Supplier<ChassisSpeeds> m_speedSupplier;

  /**
   * @param turret        Turret subsystem to command
   * @param poseSupplier  Fused pose supplier (vision + odometry) from swerve drive
   * @param speedSupplier Field-relative chassis speeds supplier from swerve drive
   */
  public TurretTrackingCommand(Turret turret,
                       Supplier<Pose2d> poseSupplier,
                       Supplier<ChassisSpeeds> speedSupplier) {
    m_turret        = turret;
    m_poseSupplier  = poseSupplier;
    m_speedSupplier = speedSupplier;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d        pose   = m_poseSupplier.get();
    ChassisSpeeds speeds = m_speedSupplier.get();

    // Odometry and vision report where the robot was a short time ago.
    // We project the robot forward by kLatencySeconds so our angle calculation
    // is based on where the robot will actually be when the ball leaves the shooter.
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(pose.getRotation());

    Pose2d futurePos = new Pose2d(
        pose.getTranslation().plus(robotVel.times(kLatencySeconds)),
        pose.getRotation().plus(
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * kLatencySeconds)
        )
    );


    // turret.getDistanceToTarget() measures from the turret pivot (not the robot
    // center) to the active target, using the offset defined in the Turret subsystem.
    // This is the single source of truth for distance across all commands.
    // If the distance is suspiciously small the pose estimator is giving bad data —
    // skip this loop rather than snapping the turret to a garbage angle.
    double distance = m_turret.getDistanceToTarget(futurePos);

    if (distance < 0.1) {
      Logger.recordOutput("TurretCommand/Warning", "Distance < 0.1m — skipping loop, check pose estimator");
      return;
    }

    // Blackbox picks the correct target (Hub or Feed station) based on which
    // zone the robot is currently in, and mirrors it to the correct alliance side.
    // We also select which time-of-flight table matches the active target type.
    Translation2d target = Blackbox.getActiveTarget(futurePos);
    Translation2d turretPivotField = futurePos.getTranslation()
        .plus(Constants.Turret.kTurretOffset.rotateBy(futurePos.getRotation()));
    Translation2d toTarget = target.minus(turretPivotField);

    TreeMap<Double, ShooterParams> activeTable;
    if (Blackbox.getActiveTarget(futurePos)==Constants.ScoringLocation.kHubPosition) {
      activeTable = hubTable;
    } else {
      activeTable = feedTable;
    }

    // A ball launched from a moving robot inherits the robot's velocity. To make
    // the ball still arrive at the target, we subtract the robot's velocity
    // (scaled by time-of-flight) from the raw target vector. Time-of-flight is
    // interpolated per distance from the same table used in ShootOnMoveCommand
    // so the lead scales correctly at every range rather than using a single average.
    // When the robot is stationary robotVel ≈ 0 so this offset ≈ 0 and the turret
    // just points straight at the target, which is correct for stationary shots.
    double timeOfFlight       = interpolate(distance, activeTable).timeOfFlight;
    Translation2d leadOffset  = robotVel.times(timeOfFlight);
    Translation2d compensatedVec = toTarget.minus(leadOffset);

    // Convert the compensated vector to a field-relative angle, then subtract
    // the robot's heading to get the turret-relative angle. Wrap to -180..180
    // so the turret never tries to spin past its physical wire stop.
    double fieldAngleToGoal = compensatedVec.getAngle().getDegrees();
    double rawTurretAngle   = MathUtil.inputModulus(
        fieldAngleToGoal - futurePos.getRotation().getDegrees(), -180, 180);

    m_turret.setSetpoint(rawTurretAngle);

    Logger.recordOutput("TurretCommand/RobotPose", pose);
    Logger.recordOutput("TurretCommand/FuturePos", futurePos);
    Logger.recordOutput("TurretCommand/TurretPivotX", turretPivotField.getX());
    Logger.recordOutput("TurretCommand/TurretPivotY", turretPivotField.getY());
    Logger.recordOutput("TurretCommand/TargetX", target.getX());
    Logger.recordOutput("TurretCommand/TargetY", target.getY());
    Logger.recordOutput("TurretCommand/Distance", distance);
    Logger.recordOutput("TurretCommand/TimeOfFlight", timeOfFlight);
    Logger.recordOutput("TurretCommand/FieldAngleToGoal", fieldAngleToGoal);
    Logger.recordOutput("TurretCommand/TurretAngle", rawTurretAngle);
    Logger.recordOutput("TurretCommand/RobotVelX", robotVel.getX());
    Logger.recordOutput("TurretCommand/RobotVelY", robotVel.getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Linearly interpolates shooter parameters (RPM, time-of-flight) for a distance
   * between two table entries. Returns the nearest edge value if distance is out of range.
   * Identical to the interpolate helper in ShootOnMoveCommand.
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
}