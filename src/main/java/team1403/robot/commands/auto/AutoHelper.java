package team1403.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import team1403.robot.util.AutoUtil;
import team1403.robot.util.CougarUtil;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Intake;
import team1403.robot.subsystems.IntakeWrist;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;
import com.pathplanner.lib.auto.NamedCommands;

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
    
    public static Command getAutoAlignTest(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("Auto Aim Command"),
            NamedCommands.getCommand("Shoot Command")            
        );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    public static Command getIntakeJiggleTest(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            Commands.race(
                    Commands.sequence(
                        NamedCommands.getCommand("Wrist Up Command"),
                        Commands.waitSeconds(1.5),
                        NamedCommands.getCommand("Wrist Down Command"),
                        Commands.waitSeconds(1.5)
                ).repeatedly()
            )   
        );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

                     
    public static Command getStationaryShoot(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("Wrist Down Command"),
            AutoUtil.loadPathPlannerPath("StationaryPt1", m_swerve, true),
            NamedCommands.getCommand("Shoot Command"),
            Commands.waitSeconds(10),
            NamedCommands.getCommand("Decelerate Shooter Flywheel Command")
            
        );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    public static Command getStationaryShootAutoAlign(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("Wrist Down Command"),
            AutoUtil.loadPathPlannerPath("StationaryPt1", m_swerve, true),
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

    public static Command getHumanPlayer(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                NamedCommands.getCommand("Wrist Down Command"),
                AutoUtil.loadPathPlannerPath("HumanPlayerPt1", m_swerve, true), 
                Commands.waitSeconds(4),
                AutoUtil.loadPathPlannerPath("HumanPlayerPt2", m_swerve, true), 
                NamedCommands.getCommand("Shoot Command"),
                Commands.waitSeconds(10),
                NamedCommands.getCommand("Decelerate Shooter Flywheel Command")
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

    public static Command getHumanPlayerAutoAlign(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                NamedCommands.getCommand("Wrist Down Command"),
                AutoUtil.loadPathPlannerPath("HumanPlayerPt1", m_swerve, true), 
                Commands.waitSeconds(4),
                AutoUtil.loadPathPlannerPath("HumanPlayerPt2", m_swerve, true), 
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

    public static Command getCenterHuman(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(1),
                    NamedCommands.getCommand("Wrist Down Command")
                ),
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("Center+HumanPt1", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("Center+HumanPt2", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.deadline(
                    Commands.waitSeconds(10),
                    NamedCommands.getCommand("Shoot Command"),
                    Commands.sequence(
                        NamedCommands.getCommand("Wrist Down Command"),
                        Commands.waitSeconds(1.5),
                        NamedCommands.getCommand("Wrist Up Command"),
                        Commands.waitSeconds(1.5)
                    ).repeatedly()
                )
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

    // public static Command getRightSweep(SwerveSubsystem m_swerve) {
    //     try {
    //         return Commands.sequence(
    //             Commands.deadline(
    //                 Commands.waitSeconds(1),
    //                 NamedCommands.getCommand("Wrist Down Command")
    //             ),
    //             Commands.parallel(
    //                 AutoUtil.loadPathPlannerPath("RightSweepPt1", m_swerve, true),
    //                 NamedCommands.getCommand("Intake Command")
    //             ),
    //             Commands.parallel(
    //                 AutoUtil.loadPathPlannerPath("RightSweepPt2", m_swerve, true),
    //                 NamedCommands.getCommand("Intake Command")
    //             ),
    //             Commands.deadline(
    //                 Commands.waitSeconds(10),
    //                 NamedCommands.getCommand("Shoot Command"),
    //                 Commands.sequence(
    //                     NamedCommands.getCommand("Wrist Down Command"),
    //                     Commands.waitSeconds(1.5),
    //                     NamedCommands.getCommand("Wrist Up Command"),
    //                     Commands.waitSeconds(1.5)
    //                 ).repeatedly()
    //             ),
    //             Commands.parallel(
    //                 AutoUtil.loadPathPlannerPath("RightSweepPt3", m_swerve, true),
    //                 NamedCommands.getCommand("Intake Command")
    //             ),
    //             Commands.parallel(
    //                 AutoUtil.loadPathPlannerPath("RightSweepPt4", m_swerve, true),
    //                 NamedCommands.getCommand("Intake Command")
    //             ),
    //             Commands.deadline(
    //                 Commands.waitSeconds(10),
    //                 NamedCommands.getCommand("Shoot Command"),
    //                 Commands.sequence(
    //                     NamedCommands.getCommand("Wrist Down Command"),
    //                     Commands.waitSeconds(1.5),
    //                     NamedCommands.getCommand("Wrist Up Command"),
    //                     Commands.waitSeconds(1.5)
    //                 ).repeatedly()
    //             )
    //         );
    //     } catch (Exception e) {
    //         System.err.println("Could not load auto: " + e.getMessage());
    //         return Commands.none();
    //     }
    // }

    // public static Command getLeftSweep(SwerveSubsystem m_swerve) {
    //     try {
    //         return Commands.sequence(
    //                 AutoUtil.loadPathPlannerPath("LeftSweepPt1", m_swerve, true), // fix names tmr
    //                 AutoUtil.loadPathPlannerPath("LeftSweepPt2", m_swerve, true),
    //                 Commands.waitSeconds(5),
    //                 // Add in the shoot command
    //                 AutoUtil.loadPathPlannerPath("LeftSweepPt3", m_swerve, true),
    //                 AutoUtil.loadPathPlannerPath("LeftSweepPt4", m_swerve, true),
    //                 Commands.waitSeconds(5)
    //         // Add in the shoot command
    //         );
    //     } catch (Exception e) {
    //         System.err.println("Could not load auto: " + e.getMessage());
    //         return Commands.none();
    //     }
    // }

    // public static Command getCenterDepot(SwerveSubsystem m_swerve) {
    //     try {
    //         return Commands.sequence(
    //                 AutoUtil.loadPathPlannerPath("Center+DepotPt1", m_swerve, true), // fix names tmr
    //                 AutoUtil.loadPathPlannerPath("Center+DepotPt2", m_swerve, true),
    //                 Commands.waitSeconds(5)
    //         // Add in the shoot command
    //         // later add in the parellel squence to intake and shoot at the same time at the
    //         // depot
    //         );
    //     } catch (Exception e) {
    //         System.err.println("Could not load auto: " + e.getMessage());
    //         return Commands.none();
    //     }
    // }

}