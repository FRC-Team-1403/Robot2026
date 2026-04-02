package team1403.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.commands.WristWiggleCommand;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;
import team1403.robot.util.AutoUtil;
import team1403.robot.util.CougarUtil;

public class AutoHelper {

    public static Command alignToStartingPose(SwerveSubsystem m_swerve, String pathName) {
        return m_swerve.defer(() -> {
            Pose2d target = AutoUtil.getStartingPose(pathName);
            if (target == null)
                return Commands.none();
            return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
        });
    }

    public static Command getMoveAuto(SwerveSubsystem m_swerve) {
        return m_swerve.defer(() -> {
            Pose2d target = m_swerve.getPose();
            if (CougarUtil.getAlliance() == Alliance.Red)
                target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.kZero, 1);
            else
                target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.k180deg, 1);
            return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
        });
    }

    public static Command getStationaryCenterShootAutoAlign(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("Wrist Down Command"),
            AutoUtil.loadPathPlannerPath("StationaryHub", m_swerve, true),
            NamedCommands.getCommand("Auto Aim Command"),
            NamedCommands.getCommand("Shoot Command"),
            Commands.waitSeconds(10),
            NamedCommands.getCommand("Decelerate Shooter Flywheel Command")
            
        );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    public static Command getMiddleHubDepotEndTrench(SwerveSubsystem m_swerve){
    try{
       return Commands.sequence(
            NamedCommands.getCommand("Wrist Down Command"),
            AutoUtil.loadPathPlannerPath("MiddleHubDepotPt1", m_swerve, true),
            Commands.race(
                AutoUtil.loadPathPlannerPath("MiddleHubDepotPt2", m_swerve, true),
                NamedCommands.getCommand("Intake Command")
            ),
            Commands.race(
                Commands.waitSeconds(2),
                NamedCommands.getCommand("Intake Command")
            ),
            AutoUtil.loadPathPlannerPath("MiddleHubDepotPt3Trench", m_swerve, true),
            NamedCommands.getCommand("Auto Aim Command"),
            Commands.race(
                NamedCommands.getCommand("Shoot Command"),
                Commands.waitSeconds(9),
                Commands.sequence(
                    NamedCommands.getCommand("Wrist Up Command"),
                    Commands.waitSeconds(0.35),
                    NamedCommands.getCommand("Wrist Down Command Jiggle")
                ).repeatedly()
            ),     
            NamedCommands.getCommand("Decelerate Shooter Flywheel Command")
        );
    } catch (Exception e) {
        System.err.println("Could not load auto: " + e.getMessage());
        return Commands.none(); 
    }
}

    public static Command getMiddleHubDepotEndHub(SwerveSubsystem m_swerve){
        try{
        return Commands.sequence(
                NamedCommands.getCommand("Wrist Down Command"),
                AutoUtil.loadPathPlannerPath("MiddleHubDepotPt1", m_swerve, true),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("MiddleHubDepotPt2", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    Commands.waitSeconds(2),
                    NamedCommands.getCommand("Intake Command")
                ),
                AutoUtil.loadPathPlannerPath("MiddleHubDepotPt3Hub", m_swerve, true),
                NamedCommands.getCommand("Auto Aim Command"),
                Commands.race(
                    NamedCommands.getCommand("Shoot Command"),
                    Commands.waitSeconds(9),
                    Commands.sequence(
                        NamedCommands.getCommand("Wrist Up Command"),
                        Commands.waitSeconds(0.35),
                        NamedCommands.getCommand("Wrist Down Command Jiggle")
                    ).repeatedly()
                ),    
                NamedCommands.getCommand("Decelerate Shooter Flywheel Command")
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    public static Command getLeftTrenchDoubleSweep(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt1", m_swerve, true),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        NamedCommands.getCommand("IntakeWrist Down Command")
                    )
                ),
                Commands.parallel(
                    Commands.sequence(
                        AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt2", m_swerve, true),
                        AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt3", m_swerve, true),
                        AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt4", m_swerve, true)
                    ),
                    NamedCommands.getCommand("intake Command")
                ),
                Commands.race(
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command"),
                    Commands.waitSeconds(5.0)
                ),

                AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt5", m_swerve, true),

                Commands.parallel(
                    Commands.sequence(
                        AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt6", m_swerve, true),
                        AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt7", m_swerve, true),
                        AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt8", m_swerve, true)
                    ),
                    NamedCommands.getCommand("intake Command")
                ),
                Commands.race(
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command")
                )
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }
}