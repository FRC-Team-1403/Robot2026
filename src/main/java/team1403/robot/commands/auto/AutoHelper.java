package team1403.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.swerve.util.AutoUtil;
import team1403.robot.swerve.util.CougarUtil;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class AutoHelper {

    public static Command alignToStartingPose(SwerveSubsystem m_swerve, String pathName) {
        return m_swerve.defer(() -> {
            Pose2d target = AutoUtil.getStartingPose(pathName);
            if (target == null) return Commands.none();
            return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
        });
    }

    public static Command getMoveAuto(SwerveSubsystem m_swerve) {
        return m_swerve.defer(() -> {
            Pose2d target = m_swerve.getPose();
            if(CougarUtil.getAlliance() == Alliance.Red)
                target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.kZero, 1);
            else
                target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.k180deg, 1);
            return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
        });
    }

    public static Command getDepoAuto(SwerveSubsystem m_swerve){
        try {
            return Commands.sequence(
                AutoUtil.loadPathPlannerPath("Deop", m_swerve)
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

}