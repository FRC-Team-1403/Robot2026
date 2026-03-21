package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;
import team1403.robot.util.Blackbox;

public class AutoAlignCommand extends Command {
    private final SwerveSubsystem m_swerve;

    private final ProfiledPIDController m_rotationPID = new ProfiledPIDController(5, 0, 0,
        new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate, 14));

    private double m_targetAngle;

    public AutoAlignCommand(SwerveSubsystem swerve) {
        m_swerve = swerve;
        m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        m_rotationPID.setTolerance(0.08, 0.15);
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        Pose2d pose = m_swerve.getPose().transformBy(
            new Transform2d(Constants.Turret.kTurretOffset, Rotation2d.kCCW_90deg.plus(Constants.Turret.rotationCorrectionOffset))
        );

        Translation2d turretPivotField = pose.getTranslation();
        Translation2d target = Blackbox.getActiveTarget(pose);

        double deltaX = target.getX() - turretPivotField.getX();
        double deltaY = target.getY() - turretPivotField.getY();

        m_targetAngle = MathUtil.angleModulus(Math.atan2(deltaY, deltaX));

        double currentAngle = MathUtil.angleModulus(pose.getRotation().getRadians());
        m_rotationPID.reset(currentAngle);
        m_rotationPID.setGoal(m_targetAngle);
    }

    @Override
    public void execute() {
        Pose2d pose = m_swerve.getPose().transformBy(
            new Transform2d(Constants.Turret.kTurretOffset, Rotation2d.kCCW_90deg.plus(Constants.Turret.rotationCorrectionOffset))
        );

        double robotHeading = MathUtil.angleModulus(pose.getRotation().getRadians());

        double angular = m_rotationPID.calculate(robotHeading, m_targetAngle);

        m_swerve.drive(new ChassisSpeeds(0, 0, angular));
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return m_rotationPID.atGoal();
    }
}