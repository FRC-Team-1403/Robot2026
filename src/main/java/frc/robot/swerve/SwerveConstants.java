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
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.8);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
    // Gear ratios: (motor rotations) / (output rotations)
    public static final double kFirstDriveStage = (15.0 / 45.0);
    public static final double kDriveGearRatio = (14.0 / 50.0) * (28.0 / 16.0) *  kFirstDriveStage;
    public static final double kSteerGearRatio = (15.0 / 32.0) * (10.0 / 60.0);
    
    // Encoder conversions: raw units → meters or radians
    public static final double kDrivePositionConversionFactor = kWheelCircumferenceMeters * kDriveGearRatio;
    public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60.0;
    public static final double kSteerPositionConversionFactor = 2 * Math.PI * kSteerGearRatio;
    public static final double kSteerVelocityConversionFactor = kSteerPositionConversionFactor / 60.0;
    
    // Steer PID (position control)
    public static final double kPSteer = 0.75;
    public static final double kISteer = 0.0;
    public static final double kDSteer = 0.06;
    
    // Current limits (amps)
    public static final int kDriveCurrentLimit = 45;
    public static final int kSteerCurrentLimit = 25;
    
    // Robot dimensions (center to center of wheels)
    public static final double kWheelWidth = Units.inchesToMeters(23);
    public static final double kWheelLength = Units.inchesToMeters(24);

    //NEO max RPM
    public static final double kDriveMotorMaxRPM = 5676;

    // Maximum speeds
    public static final double kMaxSpeed = kDriveMotorMaxRPM * kDrivePositionConversionFactor / 60.0; 
;  // ~10 mph
    public static final double kMaxAngularSpeed = (kMaxSpeed / Math.hypot(kWheelWidth / 2.0, kWheelLength / 2.0));  // 11.96207492071159 rad/s
    
    // Drive PID + Feedforward (velocity control)
    public static final double kPDrive = 0.04;
    public static final double kIDrive = 0.0;
    public static final double kDDrive = 0.0;
    public static final double kSDrive = 0;  // Static friction //tune using sysid (volts)
    public static final double kVDrive = 12/kMaxSpeed;  //check kMax // Volts per m/s
    public static final double kADrive = 0;  // Acceleration //tune using sysid (volts)

    // Kinematics - defines module positions relative to robot center
    // Order: Front Left, Front Right, Back Left, Back Right
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelLength / 2.0, kWheelWidth / 2.0),   // FL
        new Translation2d(kWheelLength / 2.0, -kWheelWidth / 2.0),  // FR
        new Translation2d(-kWheelLength / 2.0, kWheelWidth / 2.0),  // BL
        new Translation2d(-kWheelLength / 2.0, -kWheelWidth / 2.0)  // BR
    );
    
    // PathPlanner PID gains
    public static final PIDConstants kTranslationPID = new PIDConstants(5.6, 0, 0);
    public static final PIDConstants kRotationPID = new PIDConstants(2.8, 0, 0);
    /**
     * Hardware IDs and calibration for each module.
     */
    public static final class ModuleConstants {
        
        // Front Left
        public static final int kFrontLeftDriveID = 13;
        public static final int kFrontLeftSteerID = 12;
        public static final int kFrontLeftEncoderID = 22; 
        public static final double kFrontLeftOffset = -Math.PI + 0.082834967179;  // Calibrate: align wheel forward, set offset = -rawValue
        public static final boolean kFrontLeftDriveInverted = true; //fix
        
        // Front Right
        public static final int kFrontRightDriveID = 9;
        public static final int kFrontRightSteerID = 8;
        public static final int kFrontRightEncoderID = 20;
        public static final double kFrontRightOffset = -0.55 - 3.219825673771961 - Math.PI;
        public static final boolean kFrontRightDriveInverted = true; //fix
        
        // Back Left
        public static final int kBackLeftDriveID = 11;
        public static final int kBackLeftSteerID = 10;
        public static final int kBackLeftEncoderID = 21; 
        public static final double kBackLeftOffset = -0.026077673394056;
        public static final boolean kBackLeftDriveInverted = true; //fix
        
        // Back Right
        public static final int kBackRightDriveID = 7;
        public static final int kBackRightSteerID = 6;
        public static final int kBackRightEncoderID = 23; 
        public static final double kBackRightOffset = 1.25 - 6.17273869045182 - 2 * Math.PI;
        public static final boolean kBackRightDriveInverted = true; //fix
    }
}