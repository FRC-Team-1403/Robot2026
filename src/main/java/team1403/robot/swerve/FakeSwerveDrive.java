package team1403.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

public class FakeSwerveDrive extends SubsystemBase {

  private static final double kMaxSpeed  = 4.0; // m/s
  private static final double kDeadband  = 0.08;

  private final CommandXboxController m_controller = new CommandXboxController(0);

  private Pose2d        m_pose          = new Pose2d(2.0, 2.0, Rotation2d.kZero);
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private double        m_lastTime      = Timer.getFPGATimestamp();

  public Pose2d        getPose()         { return m_pose; }
  public ChassisSpeeds getChassisSpeeds(){ return m_chassisSpeeds; }

  @Override
  public void periodic() {
    // Read left stick: axis 0 = strafe, axis 1 = forward (negated — WPILib convention)
    double vx = -MathUtil.applyDeadband(m_controller.getLeftY(), kDeadband) * kMaxSpeed;
    double vy = -MathUtil.applyDeadband(m_controller.getLeftX(), kDeadband) * kMaxSpeed;
    m_chassisSpeeds = new ChassisSpeeds(vx, vy, 0);

    // Integrate position (robot is non-rotating for simplicity)
    double now = Timer.getFPGATimestamp();
    double dt  = now - m_lastTime;
    m_lastTime = now;

    m_pose = new Pose2d(
        new Translation2d(m_pose.getX() + vx * dt, m_pose.getY() + vy * dt),
        m_pose.getRotation());

    Logger.recordOutput("Field/Robot", m_pose);
  }
}
