package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.ShooterTable;

import java.util.function.Supplier;

public class ShootOnMoveCommand extends Command {

  // Arbitrary fixed goal pose — change this to wherever your hub is
  private static final Pose2d kGoalPose = new Pose2d(8.27, 4.105, Rotation2d.kZero);

  private static final Translation2d kTurretOffset  = new Translation2d(0.1, 0.05);
  private static final double        kLatencySeconds = 0.15;
  private static final int           kTrajectoryPoints = 20;

  private final Shooter                 m_shooter;
  private final ShooterHood             m_hood;
  private final Turret                  m_turret;
  private final Supplier<Pose2d>        m_poseSupplier;
  private final Supplier<ChassisSpeeds> m_speedSupplier;
  private final ShooterTable            m_table;

  public ShootOnMoveCommand(
      Shooter shooter, ShooterHood hood, Turret turret,
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier,
      ShooterTable table)
  {
    m_shooter       = shooter;
    m_hood          = hood;
    m_turret        = turret;
    m_poseSupplier  = poseSupplier;
    m_speedSupplier = speedSupplier;
    m_table         = table;
    addRequirements(shooter, hood, turret);
  }

  @Override
  public void execute() {
    Pose2d        pose   = m_poseSupplier.get();
    ChassisSpeeds speeds = m_speedSupplier.get();
    Translation2d goal   = kGoalPose.getTranslation();

    // Latency compensation
    Translation2d fieldVel = new Translation2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(pose.getRotation());
    Pose2d futurePose = new Pose2d(
        pose.getTranslation().plus(fieldVel.times(kLatencySeconds)),
        pose.getRotation().plus(Rotation2d.fromRadians(
            speeds.omegaRadiansPerSecond * kLatencySeconds)));

    // Turret pivot offset
    Translation2d turretPivot = futurePose.getTranslation()
        .plus(kTurretOffset.rotateBy(futurePose.getRotation()));

    Translation2d toGoalField = goal.minus(turretPivot);
    double distance = toGoalField.getNorm();

    // Always log goal and robot so AdvantageScope shows something even if out of range
    Logger.recordOutput("Field/Goal",  kGoalPose);
    Logger.recordOutput("Field/Robot", pose);

    if (distance < 0.5) {
      clearViz();
      return;
    }

    // Robot-relative vector for table lookup
    Translation2d toGoalRobot = toGoalField.rotateBy(futurePose.getRotation().unaryMinus());

    ShooterTable.Result result = m_table.lookup(
        toGoalRobot.getX(), toGoalRobot.getY(),
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    if (!result.valid) {
      Logger.recordOutput("SOTM/Warning", "out of table range");
      clearViz();
      return;
    }

    double turretFieldDeg = futurePose.getRotation().getDegrees()
                          + MathUtil.inputModulus(result.turretAngle, -180, 180);

    // Hardware
    m_shooter.setFlywheelTargetRPM(result.rpm);
    m_hood.setSetpoint(result.hoodAngle);
    m_turret.setSetpoint(MathUtil.inputModulus(result.turretAngle, -180, 180));

    // AdvantageScope field logs
    Logger.recordOutput("Field/TurretArrow",
        new Pose2d(turretPivot, Rotation2d.fromDegrees(turretFieldDeg)));
    Logger.recordOutput("Field/BallLanding",
        new Pose2d(goal, Rotation2d.kZero));
    Logger.recordOutput("Field/ShotLine",
        buildShotLine(turretPivot, goal));

    // Scalar logs
    Logger.recordOutput("SOTM/Distance",     distance);
    Logger.recordOutput("SOTM/RPM",          result.rpm);
    Logger.recordOutput("SOTM/HoodAngle",    result.hoodAngle);
    Logger.recordOutput("SOTM/TurretAngle",  result.turretAngle);
    Logger.recordOutput("SOTM/ReadyToShoot",
        m_shooter.isFlywheelAtSpeed() && m_hood.atSetpoint() && m_turret.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    clearViz();
  }

  private void clearViz() {
    Logger.recordOutput("Field/TurretArrow", new Pose2d[]{});
    Logger.recordOutput("Field/ShotLine",    new Pose2d[]{});
    Logger.recordOutput("Field/BallLanding", new Pose2d[]{});
  }

  private static Pose2d[] buildShotLine(Translation2d from, Translation2d to) {
    Pose2d[] pts = new Pose2d[kTrajectoryPoints];
    Translation2d delta = to.minus(from);
    Rotation2d dir = delta.getAngle();
    for (int i = 0; i < kTrajectoryPoints; i++) {
      double t = (double) i / (kTrajectoryPoints - 1);
      pts[i] = new Pose2d(from.getX() + delta.getX() * t,
                          from.getY() + delta.getY() * t, dir);
    }
    return pts;
  }
}
