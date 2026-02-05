package frc.robot.swerve;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Constants for swerve drive system - all measurements, gains, and hardware IDs.
 */
public final class SwerveConstants {
    
    // Physical measurements
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
    // Gear ratios: (motor rotations) / (output rotations)
    public static final double kDriveGearRatio = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double kSteerGearRatio = (15.0 / 32.0) * (10.0 / 60.0);
    
    // Encoder conversions: raw units → meters or radians
    public static final double kDrivePositionConversionFactor = kWheelCircumferenceMeters * kDriveGearRatio;
    public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60.0;
    public static final double kSteerPositionConversionFactor = 2 * Math.PI * kSteerGearRatio;
    public static final double kSteerVelocityConversionFactor = kSteerPositionConversionFactor / 60.0;
    
    // Maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;  // ~10 mph
    public static final double kMaxAngularSpeed = 2 * Math.PI;  // 1 rotation/second
    
    // Drive PID + Feedforward (velocity control)
    public static final double kPDrive = 0.1;
    public static final double kIDrive = 0.0;
    public static final double kDDrive = 0.0;
    public static final double kSDrive = 0.0;  // Static friction
    public static final double kVDrive = 2.5;  // Volts per m/s
    public static final double kADrive = 0.0;  // Acceleration
    
    // Steer PID (position control)
    public static final double kPSteer = 1.0;
    public static final double kISteer = 0.0;
    public static final double kDSteer = 0.1;
    
    // Current limits (amps)
    public static final int kDriveCurrentLimit = 40;
    public static final int kSteerCurrentLimit = 30;
    
    // Robot dimensions (center to center of wheels)
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.5);
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.5);
    
    // Kinematics - defines module positions relative to robot center
    // Order: Front Left, Front Right, Back Left, Back Right
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),   // FL
        new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),  // FR
        new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2),  // BL
        new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2)  // BR
    );
    
    // PathPlanner PID gains
    public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);
    
    /**
     * Hardware IDs and calibration for each module.
     */
    public static final class ModuleConstants {
        
        // Front Left
        public static final int kFrontLeftDriveID = 1;
        public static final int kFrontLeftSteerID = 2;
        public static final int kFrontLeftEncoderID = 9;
        public static final double kFrontLeftOffset = 0.0;  // Calibrate: align wheel forward, set offset = -rawValue
        public static final boolean kFrontLeftDriveInverted = false;
        
        // Front Right
        public static final int kFrontRightDriveID = 3;
        public static final int kFrontRightSteerID = 4;
        public static final int kFrontRightEncoderID = 10;
        public static final double kFrontRightOffset = 0.0;
        public static final boolean kFrontRightDriveInverted = true;
        
        // Back Left
        public static final int kBackLeftDriveID = 5;
        public static final int kBackLeftSteerID = 6;
        public static final int kBackLeftEncoderID = 11;
        public static final double kBackLeftOffset = 0.0;
        public static final boolean kBackLeftDriveInverted = false;
        
        // Back Right
        public static final int kBackRightDriveID = 7;
        public static final int kBackRightSteerID = 8;
        public static final int kBackRightEncoderID = 12;
        public static final double kBackRightOffset = 0.0;
        public static final boolean kBackRightDriveInverted = true;
    }
}