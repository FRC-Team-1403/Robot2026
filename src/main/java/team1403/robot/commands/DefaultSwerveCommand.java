package team1403.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.swerve.*;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Zone;

public class DefaultSwerveCommand extends Command {

  private final SwerveSubsystem m_drivetrainSubsystem;
  private final CommandXboxController m_driverController;

  // Input suppliers
  private final DoubleSupplier m_verticalTranslationSupplier;
  private final DoubleSupplier m_horizontalTranslationSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final BooleanSupplier m_xModeSupplier;
  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_driverSnipingMode;
  private final DoubleSupplier m_operatorSnipingMode;
  private final BooleanSupplier m_autoAim;
  private final BooleanSupplier m_robotRelativeMode;
  private final BooleanSupplier m_resetSupplier;

  // Field-relative tracking
  private final Debouncer m_robotRelativeDebouncer =
      new Debouncer(0.3, DebounceType.kFalling);

  private boolean m_isFieldRelative = true;
  private boolean m_previousReset = false;
  private double m_speedLimiter = 0.5;

  // Persistent pipeline state
  private Translation2d m_J_trans_filt = new Translation2d();
  private double m_J_omega_filt = 0.0;

  private Translation2d m_e_trans_last = new Translation2d();
  private double m_e_omega_last = 0.0;

  private Translation2d m_u_trans_exec_last = new Translation2d();
  private double m_u_omega_exec_last = 0.0;

  // Constants
  private static final double kDeadband = 0.05;
  private static final double kDtLoop = Constants.kLoopTime;

  // Dirty-derivative time constant (seconds): controls counter-braking tail duration (~3 * tau)
  private static final double kDerivTau = 0.05;

  // Intent lead gains
  private static final double K_A = 0.05;
  private static final double K_A_omega = 0.05;

  public DefaultSwerveCommand(
      SwerveSubsystem drivetrain,
      CommandXboxController driverController,
      DoubleSupplier horizontalTranslationSupplier,
      DoubleSupplier verticalTranslationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier xModeSupplier,
      BooleanSupplier robotRelativeSupplier,
      DoubleSupplier speedSupplier,
      DoubleSupplier driverSnipingMode,
      DoubleSupplier operatorSnipingMode,
      BooleanSupplier autoAim,
      BooleanSupplier resetSupplier) {

    m_drivetrainSubsystem = drivetrain;
    m_driverController = driverController;

    m_verticalTranslationSupplier = verticalTranslationSupplier;
    m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    m_rotationSupplier = rotationSupplier;

    m_speedSupplier = speedSupplier;
    m_xModeSupplier = xModeSupplier;

    m_driverSnipingMode = driverSnipingMode;
    m_operatorSnipingMode = operatorSnipingMode;

    m_robotRelativeMode = robotRelativeSupplier;
    m_autoAim = autoAim;
    m_resetSupplier = resetSupplier;

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {

    // Seed controller from actual drivetrain velocity
    ChassisSpeeds currentSpeeds = m_drivetrainSubsystem.getState().Speeds;
    Rotation2d gyroAngle = m_drivetrainSubsystem.getShallowRotation();

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, gyroAngle);

    m_u_trans_exec_last =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond);

    m_u_omega_exec_last = currentSpeeds.omegaRadiansPerSecond;

    // Clear derivative & observer memory
    m_J_trans_filt = new Translation2d();
    m_J_omega_filt = 0.0;

    m_e_trans_last = new Translation2d();
    m_e_omega_last = 0.0;

    m_previousReset = m_resetSupplier.getAsBoolean();
  }

  @Override
  public void execute() {

    // Field-relative mode
    m_isFieldRelative =
        m_robotRelativeDebouncer.calculate(!m_robotRelativeMode.getAsBoolean());

    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);

    // Autonomous protection
    if (DriverStation.isAutonomousEnabled()) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds());
      return;
    }

    // X-lock mode
    if (m_xModeSupplier.getAsBoolean()) {
      m_drivetrainSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());

      m_u_trans_exec_last = new Translation2d();
      m_u_omega_exec_last = 0.0;
      m_J_trans_filt = new Translation2d();
      m_J_omega_filt = 0.0;

      if (m_driverController != null) {
        m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
        m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
      }

      return;
    }

    // Gyro reset - rising edge only
    boolean resetPressed = m_resetSupplier.getAsBoolean();

    if (resetPressed && !m_previousReset) {
      var alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        m_drivetrainSubsystem.resetRotation(Rotation2d.kPi);
      } else {
        m_drivetrainSubsystem.resetRotation(Rotation2d.kZero);
      }

      m_J_trans_filt = new Translation2d();
      m_J_omega_filt = 0.0;
      m_e_trans_last = new Translation2d();
      m_e_omega_last = 0.0;
    }

    m_previousReset = resetPressed;

    // ==========================================
    // DYNAMIC ZONE SPEED LIMITER
    // ==========================================

    Pose2d currentPose = m_drivetrainSubsystem.getPose();
    Zone zone = FieldZoneUtil.getZone(currentPose);

    boolean driverSniping = m_driverSnipingMode.getAsDouble() > 0.3;
    boolean operatorSniping = m_operatorSnipingMode.getAsDouble() > 0.3;

    if (zone == Zone.MY_ALLIANCE) {
      m_speedLimiter =
          (operatorSniping || driverSniping)
              ? 0.2
              : (m_speedSupplier.getAsDouble() > 0.3 ? 0.7 : 0.5);
    } else {
      m_speedLimiter =
          (operatorSniping || driverSniping)
              ? 0.3
              : (m_speedSupplier.getAsDouble() > 0.3 ? 0.7 : 0.5);
    }

    double vMaxActive = TunerConstants.kMaxSpeed * m_speedLimiter;
    double omegaMaxActive = TunerConstants.kMaxAngularRate * m_speedLimiter;

    // ==========================================
    // STEP 1: PRE-FILTERING AND DEADBANDS
    // ==========================================

    double jRawX = m_verticalTranslationSupplier.getAsDouble();
    double jRawY = m_horizontalTranslationSupplier.getAsDouble();
    double jRawOmega = m_rotationSupplier.getAsDouble();

    double mRaw = Math.hypot(jRawX, jRawY);
    Translation2d J_trans = new Translation2d();

    if (mRaw > kDeadband) {
      double scaledMag = (mRaw - kDeadband) / (mRaw * (1.0 - kDeadband));
      J_trans = new Translation2d(jRawX * scaledMag, jRawY * scaledMag);
    }

    double J_omega = 0.0;

    if (Math.abs(jRawOmega) > kDeadband) {
      J_omega =
          Math.signum(jRawOmega)
              * (Math.abs(jRawOmega) - kDeadband)
              / (1.0 - kDeadband);
    }

    // Alliance red field-relative flip
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red && m_isFieldRelative) {
      J_trans = J_trans.times(-1.0);
    }

    // ==========================================
    // STEP 2: INTENT-SHAPED CONTROL LAW (OPEN-LOOP)
    // ==========================================

    double alpha = kDtLoop / (kDerivTau + kDtLoop);

    m_J_trans_filt =
        m_J_trans_filt.plus(J_trans.minus(m_J_trans_filt).times(alpha));
    Translation2d dJ_trans_dt =
        J_trans.minus(m_J_trans_filt).div(kDerivTau);

    m_J_omega_filt += alpha * (J_omega - m_J_omega_filt);
    double dJ_omega_dt =
        (J_omega - m_J_omega_filt) / kDerivTau;

    Translation2d u_trans_raw =
        J_trans.times(vMaxActive).plus(dJ_trans_dt.times(K_A));

    double u_omega_raw =
        (J_omega * omegaMaxActive) + (dJ_omega_dt * K_A_omega);

    // ==========================================
    // STEP 3: ACCELERATION ENVELOPE
    // ==========================================

    Translation2d delta_u_trans = u_trans_raw.minus(m_u_trans_exec_last);
    double dmag = delta_u_trans.getNorm();
    double maxVelChangeLoop = 1000.0 * kDtLoop;

    Translation2d u_trans_env =
        (dmag > 0.0)
            ? m_u_trans_exec_last.plus(
                delta_u_trans.times(Math.min(1.0, maxVelChangeLoop / dmag)))
            : u_trans_raw;

    double maxAlphaChangeLoop = 1000.0 * kDtLoop;

    double u_omega_env =
        m_u_omega_exec_last
            + MathUtil.clamp(
                u_omega_raw - m_u_omega_exec_last,
                -maxAlphaChangeLoop,
                maxAlphaChangeLoop);

    // ==========================================
    // STEP 4: ABSOLUTE VELOCITY SATURATION
    // ==========================================

    double envNorm = u_trans_env.getNorm();

    Translation2d u_trans_final =
        u_trans_env.times(
            Math.min(1.0, vMaxActive / (envNorm == 0.0 ? 1.0 : envNorm)));

    double u_omega_final =
        MathUtil.clamp(u_omega_env, -omegaMaxActive, omegaMaxActive);

// ==========================================
// STEP 5: HAPTIC OBSERVER & RUMBLE ENGINE
// ==========================================
ChassisSpeeds rawRobotVelocity = m_drivetrainSubsystem.getState().Speeds;
Rotation2d heading = m_drivetrainSubsystem.getShallowRotation();
Translation2d V_meas =
new Translation2d(
rawRobotVelocity.vxMetersPerSecond,
rawRobotVelocity.vyMetersPerSecond);
if (m_isFieldRelative) {
V_meas = V_meas.rotateBy(heading);
}
Translation2d X_trans = V_meas.div(vMaxActive);
double X_omega = rawRobotVelocity.omegaRadiansPerSecond / omegaMaxActive;
Translation2d e_trans = J_trans.minus(X_trans);
double e_omega = J_omega - X_omega;
Translation2d de_trans_dt = e_trans.minus(m_e_trans_last).div(kDtLoop);
if (m_driverController != null) {
// Left motor: sustained tracking loss
double errorMagnitude = e_trans.getNorm();
double leftRumbleIntensity = 0.0;
if (errorMagnitude > 0.15) {
leftRumbleIntensity =
MathUtil.clamp((errorMagnitude - 0.15) / 0.85, 0.0, 1.0);
}
// Right motor: derivative impact alert
double impactMagnitude = de_trans_dt.getNorm();
double rightRumbleIntensity = 0.0;
if (impactMagnitude > 4.5) {
rightRumbleIntensity =
MathUtil.clamp(impactMagnitude / 15.0, 0.0, 1.0);
}
m_driverController.setRumble(RumbleType.kLeftRumble, leftRumbleIntensity);
m_driverController.setRumble(RumbleType.kRightRumble, rightRumbleIntensity);
}
// ==========================================
// STEP 6: PERSISTENT MEMORY HANDOVER
// ==========================================
m_e_trans_last = e_trans;
m_e_omega_last = e_omega;
m_u_trans_exec_last = u_trans_final;
m_u_omega_exec_last = u_omega_final;
// ==========================================
// OUTPUT HANDOFF
// ==========================================
ChassisSpeeds outputSpeeds =
m_isFieldRelative
? ChassisSpeeds.fromFieldRelativeSpeeds(
u_trans_final.getX(),
u_trans_final.getY(),
u_omega_final,
heading)
: new ChassisSpeeds(
u_trans_final.getX(),
u_trans_final.getY(),
u_omega_final);
m_drivetrainSubsystem.drive(outputSpeeds);
Logger.recordOutput("Error", m_e_trans_last);
Logger.recordOutput("Error Angular", m_e_omega_last);
Logger.recordOutput("dy/dx Joystick", dJ_trans_dt);
}
@Override
public void end(boolean interrupted) {
m_drivetrainSubsystem.drive(new ChassisSpeeds());
if (m_driverController != null) {
m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
}
}
}
