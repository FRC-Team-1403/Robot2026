
package team1403.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class DefaultSwerveCommand extends Command {

  // ==========================================
  // CONFIGURATION AND TUNING CONSTANTS
  // ==========================================

  // Joystick input deadband
  private static final double kDeadband = 0.05;

  // Loop period in seconds
  private static final double kDtLoop = Constants.kLoopTime;

  // Joystick response curve weights
  // 0.0 = completely linear, 1.0 = completely cubic
  private static final double kCubicWeight = 0.30;
  private static final double kCubicWeightOmega = 0.35;

  // Low-pass filter time constant for derivative estimation
  private static final double kDerivTau = 0.05;

  // Intent lead gains
  // Determines how strongly changes in joystick input affect commanded velocity
  private static final double K_A = 0.05;
  private static final double K_A_omega = 0.05;

  // Maximum allowed velocity and angular velocity changes per second
  private static final double kMaxAcceleration = 1000.0;
  private static final double kMaxAngularAcceleration = 1000.0;

  // Haptic feedback thresholds
  private static final double kRumbleErrorThreshold = 0.15;
  private static final double kRumbleImpactThreshold = 4.5;
  private static final double kRumbleImpactScale = 15.0;

  // ==========================================
  // SUBSYSTEMS AND CONTROLLER INPUTS
  // ==========================================

  private final SwerveSubsystem m_drivetrainSubsystem;
  private final CommandXboxController m_driverController;

  private final DoubleSupplier m_verticalTranslationSupplier;
  private final DoubleSupplier m_horizontalTranslationSupplier;
  private final DoubleSupplier m_rotationSupplier;

  private final BooleanSupplier m_xModeSupplier;
  private final BooleanSupplier m_resetSupplier;

  // Tracks the previous reset-button state to detect a new press
  private boolean m_previousReset = false;

  // ==========================================
  // PERSISTENT CONTROL STATE
  // ==========================================

  // Filtered joystick inputs
  private Translation2d m_J_trans_filt = new Translation2d();
  private double m_J_omega_filt = 0.0;

  // Previous tracking errors for derivative calculations
  private Translation2d m_e_trans_last = new Translation2d();
  private double m_e_omega_last = 0.0;

  // Previous commanded velocities for acceleration limiting
  private Translation2d m_u_trans_exec_last = new Translation2d();
  private double m_u_omega_exec_last = 0.0;

  // ==========================================
  // CONSTRUCTOR
  // ==========================================

  public DefaultSwerveCommand(
      SwerveSubsystem drivetrain,
      CommandXboxController driverController,
      DoubleSupplier horizontalTranslationSupplier,
      DoubleSupplier verticalTranslationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier xModeSupplier,
      BooleanSupplier resetSupplier) {

    m_drivetrainSubsystem = drivetrain;
    m_driverController = driverController;

    m_verticalTranslationSupplier = verticalTranslationSupplier;
    m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    m_rotationSupplier = rotationSupplier;

    m_xModeSupplier = xModeSupplier;
    m_resetSupplier = resetSupplier;

    addRequirements(m_drivetrainSubsystem);
  }

  // ==========================================
  // JOYSTICK RESPONSE CURVE
  // ==========================================

  // Combines linear and cubic responses to provide
  // finer control at low joystick inputs.
  private static double applyResponseCurve(double normalizedMag, double cubicWeight) {
    return (1.0 - cubicWeight) * normalizedMag
        + cubicWeight * normalizedMag * normalizedMag * normalizedMag;
  }

  // ==========================================
  // COMMAND INITIALIZATION
  // ==========================================

  @Override
  public void initialize() {

    // Read the drivetrain's current velocity
    ChassisSpeeds currentSpeeds = m_drivetrainSubsystem.getState().Speeds;
    Rotation2d gyroAngle = m_drivetrainSubsystem.getShallowRotation();

    // Convert the current robot-relative velocity into field-relative coordinates
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, gyroAngle);

    // Initialize the previous commanded velocity using the actual drivetrain velocity
    m_u_trans_exec_last =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond);

    m_u_omega_exec_last = currentSpeeds.omegaRadiansPerSecond;

    // Clear filter and observer history
    m_J_trans_filt = new Translation2d();
    m_J_omega_filt = 0.0;

    m_e_trans_last = new Translation2d();
    m_e_omega_last = 0.0;

    // Prevent an already-held reset button from triggering immediately
    m_previousReset = m_resetSupplier.getAsBoolean();
  }

  // ==========================================
  // MAIN CONTROL LOOP
  // ==========================================

  @Override
  public void execute() {

    // ------------------------------------------
    // AUTONOMOUS PROTECTION
    // ------------------------------------------

    // Prevent the driver command from applying joystick control during autonomous
    if (DriverStation.isAutonomousEnabled()) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds());
      return;
    }

    // ------------------------------------------
    // X-LOCK MODE
    // ------------------------------------------

    // Lock the swerve modules in an X formation
    if (m_xModeSupplier.getAsBoolean()) {
      m_drivetrainSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());

      // Clear previous velocity and filter states
      m_u_trans_exec_last = new Translation2d();
      m_u_omega_exec_last = 0.0;
      m_J_trans_filt = new Translation2d();
      m_J_omega_filt = 0.0;

      // Disable controller rumble while X-lock is active
      if (m_driverController != null) {
        m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
        m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
      }

      return;
    }

    // ------------------------------------------
    // GYRO RESET
    // ------------------------------------------

    // Only reset heading when the reset button transitions from released to pressed
    boolean resetPressed = m_resetSupplier.getAsBoolean();

    if (resetPressed && !m_previousReset) {
      var alliance = DriverStation.getAlliance();

      // Set the starting field heading based on alliance color
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        m_drivetrainSubsystem.resetRotation(Rotation2d.kPi);
      } else {
        m_drivetrainSubsystem.resetRotation(Rotation2d.kZero);
      }

      // Clear filter and observer history after changing the gyro heading
      m_J_trans_filt = new Translation2d();
      m_J_omega_filt = 0.0;
      m_e_trans_last = new Translation2d();
      m_e_omega_last = 0.0;
    }

    m_previousReset = resetPressed;

    // ------------------------------------------
    // STEP 1: READ AND SHAPE JOYSTICK INPUTS
    // ------------------------------------------

    // Read driver translation and rotation inputs
    double jRawX = m_verticalTranslationSupplier.getAsDouble();
    double jRawY = m_horizontalTranslationSupplier.getAsDouble();
    double jRawOmega = m_rotationSupplier.getAsDouble();

    // Calculate the magnitude of the translation joystick
    double mRaw = Math.hypot(jRawX, jRawY);
    Translation2d J_trans = new Translation2d();

    // Apply a radial deadband and response curve to translation
    if (mRaw > kDeadband) {
      double clampedMag = Math.min(mRaw, 1.0);
      double normMag = (clampedMag - kDeadband) / (1.0 - kDeadband);
      double curvedMag = applyResponseCurve(normMag, kCubicWeight);

      // Preserve joystick direction while applying the shaped magnitude
      double scale = curvedMag / mRaw;
      J_trans = new Translation2d(jRawX * scale, jRawY * scale);
    }

    // Apply deadband and response curve to rotational input
    double J_omega = 0.0;

    if (Math.abs(jRawOmega) > kDeadband) {
      double clampedOmega = Math.min(Math.abs(jRawOmega), 1.0);
      double normOmega = (clampedOmega - kDeadband) / (1.0 - kDeadband);
      double curvedOmega = applyResponseCurve(normOmega, kCubicWeightOmega);

      J_omega = Math.signum(jRawOmega) * curvedOmega;
    }

    // Reverse field-relative translation inputs on the red alliance
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      J_trans = J_trans.times(-1.0);
    }

    // ------------------------------------------
    // STEP 2: FILTER INPUTS AND CALCULATE INTENT
    // ------------------------------------------

    // Calculate the low-pass filter coefficient
    double alpha = kDtLoop / (kDerivTau + kDtLoop);

    // Smooth translation input to estimate its rate of change
    m_J_trans_filt =
        m_J_trans_filt.plus(J_trans.minus(m_J_trans_filt).times(alpha));

    Translation2d dJ_trans_dt =
        J_trans.minus(m_J_trans_filt).div(kDerivTau);

    // Smooth rotational input and estimate its rate of change
    m_J_omega_filt += alpha * (J_omega - m_J_omega_filt);

    double dJ_omega_dt =
        (J_omega - m_J_omega_filt) / kDerivTau;

    // Combine joystick position and its rate of change
    // to generate the desired translational velocity
    Translation2d u_trans_raw =
        J_trans.times(TunerConstants.kMaxSpeed)
            .plus(dJ_trans_dt.times(K_A));

    // Generate the desired angular velocity
    double u_omega_raw =
        (J_omega * TunerConstants.kMaxAngularRate)
            + (dJ_omega_dt * K_A_omega);

    // ------------------------------------------
    // STEP 3: LIMIT VELOCITY CHANGES
    // ------------------------------------------

    // Calculate the difference between the new and previous translation commands
    Translation2d delta_u_trans = u_trans_raw.minus(m_u_trans_exec_last);
    double dmag = delta_u_trans.getNorm();

    // Limit how much translational velocity can change in one loop
    double maxVelChangeLoop = kMaxAcceleration * kDtLoop;

    Translation2d u_trans_env =
        (dmag > 0.0)
            ? m_u_trans_exec_last.plus(
                delta_u_trans.times(Math.min(1.0, maxVelChangeLoop / dmag)))
            : u_trans_raw;

    // Apply the same type of limit to angular velocity
    double maxAlphaChangeLoop = kMaxAngularAcceleration * kDtLoop;

    double u_omega_env =
        m_u_omega_exec_last
            + MathUtil.clamp(
                u_omega_raw - m_u_omega_exec_last,
                -maxAlphaChangeLoop,
                maxAlphaChangeLoop);

    // ------------------------------------------
    // STEP 4: ENFORCE MAXIMUM VELOCITIES
    // ------------------------------------------

    // Keep the translation command within the drivetrain's maximum speed
    double envNorm = u_trans_env.getNorm();

    Translation2d u_trans_final =
        u_trans_env.times(
            Math.min(
                1.0,
                TunerConstants.kMaxSpeed / (envNorm == 0.0 ? 1.0 : envNorm)));

    // Keep angular velocity within the drivetrain's maximum angular rate
    double u_omega_final =
        MathUtil.clamp(
            u_omega_env,
            -TunerConstants.kMaxAngularRate,
            TunerConstants.kMaxAngularRate);

    // ------------------------------------------
    // STEP 5: MEASURE TRACKING ERROR AND PROVIDE RUMBLE
    // ------------------------------------------

    // Read actual robot-relative drivetrain velocity
    ChassisSpeeds rawRobotVelocity = m_drivetrainSubsystem.getState().Speeds;
    Rotation2d heading = m_drivetrainSubsystem.getShallowRotation();

    // Convert measured translation velocity into field-relative coordinates
    Translation2d V_meas_world =
        new Translation2d(
                rawRobotVelocity.vxMetersPerSecond,
                rawRobotVelocity.vyMetersPerSecond)
            .rotateBy(heading);

    // Normalize measured velocities so they can be compared with joystick inputs
    Translation2d X_trans = V_meas_world.div(TunerConstants.kMaxSpeed);

    double X_omega =
        rawRobotVelocity.omegaRadiansPerSecond / TunerConstants.kMaxAngularRate;

    // Calculate the difference between requested and measured motion
    Translation2d e_trans = J_trans.minus(X_trans);
    double e_omega = J_omega - X_omega;

    // Calculate how quickly the translation tracking error is changing
    Translation2d de_trans_dt =
        e_trans.minus(m_e_trans_last).div(kDtLoop);

    if (m_driverController != null) {

      // Left rumble: indicates sustained difference between requested and actual motion
      double errorMagnitude = e_trans.getNorm();
      double leftRumbleIntensity = 0.0;

      if (errorMagnitude > kRumbleErrorThreshold) {
        leftRumbleIntensity =
            MathUtil.clamp(
                (errorMagnitude - kRumbleErrorThreshold) / 0.85,
                0.0,
                1.0);
      }

      // Right rumble: indicates a rapid change in tracking error
      double impactMagnitude = de_trans_dt.getNorm();
      double rightRumbleIntensity = 0.0;

      if (impactMagnitude > kRumbleImpactThreshold) {
        rightRumbleIntensity =
            MathUtil.clamp(
                impactMagnitude / kRumbleImpactScale,
                0.0,
                1.0);
      }

      m_driverController.setRumble(RumbleType.kLeftRumble, leftRumbleIntensity);
      m_driverController.setRumble(RumbleType.kRightRumble, rightRumbleIntensity);
    }

    // ------------------------------------------
    // STEP 6: SAVE STATE FOR THE NEXT LOOP
    // ------------------------------------------

    // Store current tracking errors for the next derivative calculation
    m_e_trans_last = e_trans;
    m_e_omega_last = e_omega;

    // Store current commanded velocities for the next acceleration limit calculation
    m_u_trans_exec_last = u_trans_final;
    m_u_omega_exec_last = u_omega_final;

    // ------------------------------------------
    // STEP 7: SEND COMMAND TO DRIVETRAIN
    // ------------------------------------------

    // Convert field-relative velocity commands into robot-relative speeds
    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            u_trans_final.getX(),
            u_trans_final.getY(),
            u_omega_final,
            heading);

    m_drivetrainSubsystem.drive(outputSpeeds);

    // Publish tracking information to AdvantageKit
    Logger.recordOutput("Error", m_e_trans_last);
    Logger.recordOutput("Error Angular", m_e_omega_last);
    Logger.recordOutput("dy/dx Joystick", dJ_trans_dt);
  }

  // ==========================================
  // COMMAND END
  // ==========================================

  @Override
  public void end(boolean interrupted) {

    // Stop the drivetrain when the command ends
    m_drivetrainSubsystem.drive(new ChassisSpeeds());

    // Clear controller rumble
    if (m_driverController != null) {
      m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
      m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
    }
  }
}
