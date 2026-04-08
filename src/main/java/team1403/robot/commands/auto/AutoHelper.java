package team1403.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.Constants;
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


    //shoot in front of hub -- FIX!!!
    public static Command getStationaryCenterShootAutoAlign(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("IntakeWrist Down Command"),
            AutoUtil.loadPathPlannerPath("StationaryHub", m_swerve, true),
            NamedCommands.getCommand("Shoot Command"),
            Commands.waitSeconds(10)
            
        );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    //Depo -- FIX!!!!!
    public static Command getMiddleHubDepotEndTrench(SwerveSubsystem m_swerve){
    try{
       return Commands.sequence(
            Commands.race(
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("MiddleHubDepotPt1", m_swerve, true),
                    NamedCommands.getCommand("IntakeWrist Down Command")
                ),
                NamedCommands.getCommand("Intake Command")
            ),
            Commands.race(
                 Commands.parallel(
                    AutoUtil.loadPathPlannerPath("MiddleHubDepotPt2", m_swerve, true),
                    NamedCommands.getCommand("Shoot Command")
                ),
                NamedCommands.getCommand("Intake Command")
            ), 
            Commands.race(
                AutoUtil.loadPathPlannerPath("MiddleHubDepotPt3Trench", m_swerve, true),
                NamedCommands.getCommand("Intake Command")
            ), 
            Commands.race(
                NamedCommands.getCommand("ShootCommand"),
                NamedCommands.getCommand("Wrist Wiggle Command"),
                Commands.waitSeconds(4.0)
            )
        );
    } catch (Exception e) {
        System.err.println("Could not load auto: " + e.getMessage());
        return Commands.none(); 
    }
}

    //Left trench double sweep 
    public static Command getLeftTrenchDoubleSweep(SwerveSubsystem m_swerve) {
    try {
        return Commands.sequence(
            Commands.race(
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt1", m_swerve, true),
                    NamedCommands.getCommand("IntakeWrist Down Command"),
                    Commands.sequence(
                        Commands.waitSeconds(4.0),
                        NamedCommands.getCommand("Shoot Command")
                    )
                ),
                NamedCommands.getCommand("Intake Command")
            ),
            Commands.race(
                Commands.parallel(
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command")
                ),
                Commands.waitSeconds(5.0)
            ),
            Commands.race(
                AutoUtil.loadPathPlannerPath("LeftTrenchSweepMoreLoopyPt3", m_swerve, true),
                NamedCommands.getCommand("Intake Command")
            ),
            Commands.race(
                Commands.parallel(
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command")
                ),
                Commands.waitSeconds(5.0)
            )
        );
    } catch (Exception e) {
        System.err.println("Could not load auto: " + e.getMessage());
        return Commands.none();
    }
}
    //right trench double sweep 
    public static Command getRightTrenchDoubleSweep(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.race(
                    Commands.parallel(
                        AutoUtil.loadPathPlannerPath("RightTrenchSweepPt1", m_swerve, true),
                        NamedCommands.getCommand("IntakeWrist Down Command")
                    ),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command"),
                    Commands.waitSeconds(4.0)
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("RightTrenchSweepPt2", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command"),
                    Commands.waitSeconds(4.0)
                )
        );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }
       
}