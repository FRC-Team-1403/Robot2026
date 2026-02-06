package frc.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.util.CougarUtil;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

/**
 * The default command for the swerve drivetrain subsystem.
 */
public class DefaultSwerveCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrainSubsystem;

  private final DoubleSupplier m_verticalTranslationSupplier;
  private final DoubleSupplier m_horizontalTranslationSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final BooleanSupplier m_xModeSupplier;
  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_snipingMode;
  private final BooleanSupplier m_robotRelativeMode;
  
  private final Debouncer m_robotRelativeDebouncer 
    = new Debouncer(0.3, DebounceType.kFalling);
  private boolean m_isFieldRelative = true;
  
  private SlewRateLimiter m_rotationRateLimiter;
  private double prev_horizontal = 0;
  private double prev_vertical = 0;
  private static final double kMaxVelocityChange = 13 * Constants.kLoopTime;

  private double m_speedLimiter = 0.2;
  
  // Calculate max angular rate based on max speed and robot size
  // Using the front-left module position as the radius (distance from center)
  private static final double kMaxAngularRate;
  static {
    // Module positions in meters (14 inches = 0.3556 meters)
    double moduleRadius = Math.hypot(0.3556, 0.3556); // sqrt(14^2 + 14^2) inches in meters
    // Max angular velocity = max linear speed / radius
    kMaxAngularRate = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / moduleRadius;
  }
  
  // SwerveRequest for driving
  private final SwerveRequest.FieldCentric m_fieldCentricRequest = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric m_robotCentricRequest = new SwerveRequest.RobotCentric();

  /**
   * Creates the swerve command.
   * 
   * @param drivetrain                    the instance of the CommandSwerveDrivetrain
   * @param horizontalTranslationSupplier supplies the horizontal speed of the drivetrain
   * @param verticalTranslationSupplier   supplies the vertical speed of the drivetrain
   * @param rotationSupplier              supplies the rotational speed of the drivetrain
   * @param xModeSupplier                 supplies boolean to enable X-mode (brake configuration)
   * @param robotRelativeSupplier         supplies boolean to enable robot-relative mode
   * @param speedSupplier                 supplies speed multiplier (0-1)
   * @param snipingMode                   supplies sniping mode multiplier (0-1, reduces speed)
   */
  public DefaultSwerveCommand(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier horizontalTranslationSupplier,
      DoubleSupplier verticalTranslationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier xModeSupplier,
      BooleanSupplier robotRelativeSupplier,
      DoubleSupplier speedSupplier,
      DoubleSupplier snipingMode) {
    this.m_drivetrainSubsystem = drivetrain;
    this.m_verticalTranslationSupplier = verticalTranslationSupplier;
    this.m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_speedSupplier = speedSupplier;
    this.m_xModeSupplier = xModeSupplier;
    this.m_snipingMode = snipingMode;
    this.m_robotRelativeMode = robotRelativeSupplier;
    
    m_isFieldRelative = true;
    m_rotationRateLimiter = new SlewRateLimiter(3, -3, 0);

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    prev_horizontal = prev_vertical = 0;
  }

  @Override
  public void execute() {
    // Update field-relative mode based on debounced input
    m_isFieldRelative = m_robotRelativeDebouncer.calculate(!m_robotRelativeMode.getAsBoolean());

    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);

    // Calculate speed limiter: base 30% + up to 70% based on speed supplier, reduced by sniping mode
    m_speedLimiter = 0.3 * (1.0 - m_snipingMode.getAsDouble() * 0.7) 
                   + squareNum(m_speedSupplier.getAsDouble()) * 0.7;
  
    // Don't drive during autonomous
    if (DriverStation.isAutonomousEnabled()) {
      m_drivetrainSubsystem.setControl(m_fieldCentricRequest.withVelocityX(MetersPerSecond.zero())
                                                           .withVelocityY(MetersPerSecond.zero())
                                                           .withRotationalRate(RadiansPerSecond.zero()));
      return;
    }

    // X-mode: lock wheels in X configuration for defense
    if (m_xModeSupplier.getAsBoolean()) {
      m_drivetrainSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());
      return;
    }

    // Get joystick inputs
    double horizontal = m_horizontalTranslationSupplier.getAsDouble();
    double vertical = m_verticalTranslationSupplier.getAsDouble();
    double vel_hypot = Math.hypot(horizontal, vertical);

    // Flip translation for red alliance in field-relative mode
    if (CougarUtil.getAlliance() == Alliance.Red && m_isFieldRelative) {
      horizontal *= -1;
      vertical *= -1;
    }

    // Normalize and scale translation inputs
    if (vel_hypot > 0) {
      // Clamp and apply deadband to velocity magnitude
      double velocity = MathUtil.clamp(vel_hypot, 0, 1);
      velocity = MathUtil.applyDeadband(velocity, 0.05);
      
      // Scale unit vector by speed limiter and convert to m/s
      // Using kSpeedAt12Volts as max speed
      horizontal *= velocity / vel_hypot * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * m_speedLimiter;
      vertical *= velocity / vel_hypot * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * m_speedLimiter;
    }

    // Process rotation input with deadband, squaring, and slew rate limiting
    double ang_deadband = MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), 0.05);
    double angular = m_rotationRateLimiter.calculate(squareNum(ang_deadband) * m_speedLimiter) 
                   * kMaxAngularRate;

    // Limit change in translation to prevent sudden acceleration
    // Based on Orbit's slideshow technique
    {
      double dx = horizontal - prev_horizontal;
      double dy = vertical - prev_vertical;
      double dmag = Math.hypot(dx, dy);

      if (dmag > 0) {
        double scale = MathUtil.clamp(dmag, 0, kMaxVelocityChange) / dmag;
        dx *= scale;
        dy *= scale;
        horizontal = prev_horizontal + dx;
        vertical = prev_vertical + dy;
      }

      prev_horizontal = horizontal;
      prev_vertical = vertical;
    }

    // Create and send drive command using SwerveRequest
    if (m_isFieldRelative) {
      m_drivetrainSubsystem.setControl(
        m_fieldCentricRequest
          .withVelocityX(MetersPerSecond.of(vertical))
          .withVelocityY(MetersPerSecond.of(horizontal))
          .withRotationalRate(RadiansPerSecond.of(angular))
      );
    } else {
      m_drivetrainSubsystem.setControl(
        m_robotCentricRequest
          .withVelocityX(MetersPerSecond.of(vertical))
          .withVelocityY(MetersPerSecond.of(horizontal))
          .withRotationalRate(RadiansPerSecond.of(angular))
      );
    }
  }

  /**
   * Squares the input while preserving sign for smoother joystick control.
   * 
   * @param num the input value
   * @return the squared value with original sign
   */
  private static double squareNum(double num) {
    return Math.signum(num) * Math.pow(num, 2);
  }
}