package frc.robot.swerve;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
    public static final double kDriveGearRatio = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    public static final double kSteerGearRatio = (15.0 / 32.0) * (10.0 / 60.0);
    
    public static final double kDrivePositionConversionFactor = kWheelCircumferenceMeters * kDriveGearRatio;
    public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60.0;
    
    public static final double kSteerPositionConversionFactor = 2 * Math.PI * kSteerGearRatio;
    public static final double kSteerVelocityConversionFactor = kSteerPositionConversionFactor / 60.0;
    
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    
    public static final double kPDrive = 0.1;
    public static final double kIDrive = 0.0;
    public static final double kDDrive = 0.0;
    public static final double kSDrive = 0.0;
    public static final double kVDrive = 2.5;
    public static final double kADrive = 0.0;
    
    public static final double kPSteer = 1.0;
    public static final double kISteer = 0.0;
    public static final double kDSteer = 0.1;
    
    public static final int kDriveCurrentLimit = 40;
    public static final int kSteerCurrentLimit = 30;
    
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.5);
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.5);
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
        new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
        new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2),
        new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2)
    );
    
    public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);
    
    public static final class ModuleConstants {
        public static final int kFrontLeftDriveID = 1;
        public static final int kFrontLeftSteerID = 2;
        public static final int kFrontLeftEncoderID = 9;
        public static final double kFrontLeftOffset = 0.0;
        public static final boolean kFrontLeftDriveInverted = false;
        
        public static final int kFrontRightDriveID = 3;
        public static final int kFrontRightSteerID = 4;
        public static final int kFrontRightEncoderID = 10;
        public static final double kFrontRightOffset = 0.0;
        public static final boolean kFrontRightDriveInverted = true;
        
        public static final int kBackLeftDriveID = 5;
        public static final int kBackLeftSteerID = 6;
        public static final int kBackLeftEncoderID = 11;
        public static final double kBackLeftOffset = 0.0;
        public static final boolean kBackLeftDriveInverted = false;
        
        public static final int kBackRightDriveID = 7;
        public static final int kBackRightSteerID = 8;
        public static final int kBackRightEncoderID = 12;
        public static final double kBackRightOffset = 0.0;
        public static final boolean kBackRightDriveInverted = true;
    }
}