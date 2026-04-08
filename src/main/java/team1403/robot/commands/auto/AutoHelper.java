package team1403.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.util.AutoUtil;

public class AutoHelper {
    //Shoots the preloaded 8 fuel at the hub position
    public static Command getPreloadedFuelHub(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("IntakeWrist Down Command"),
            NamedCommands.getCommand("Shoot Command")            
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    //Shoots the preloaded 8 fuel at the left trench position
    public static Command getPreloadedFuelLeftTrench(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("IntakeWrist Down Command"),
            NamedCommands.getCommand("Shoot Command")            
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    //Shoots the preloaded 8 fuel at the right trench position
    public static Command getPreloadedFuelRightTrench(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("IntakeWrist Down Command"),
            NamedCommands.getCommand("Shoot Command")            
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none(); 
        }
    }

    //Shoots balls at the hub 
    public static Command getHubDepot(SwerveSubsystem m_swerve){
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
                        AutoUtil.loadPathPlannerPath("MiddleHubDepotPt2", m_swerve, true),
                        NamedCommands.getCommand("Intake Command")    
                    ),
                    Commands.parallel(
                        NamedCommands.getCommand("Shoot Command"),
                        NamedCommands.getCommand("Wrist Wiggle Command")
                    )
                );
            } catch (Exception e) {
                System.err.println("Could not load auto: " + e.getMessage());
                return Commands.none(); 
            }
        }

    //Left ouble sweep 
    public static Command getLeftTrenchDoubleSweep(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.race(
                    Commands.parallel(
                        AutoUtil.loadPathPlannerPath("LeftTrenchDoubleSweepPt1", m_swerve, true),
                        NamedCommands.getCommand("IntakeWrist Down Command")
                    ),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    Commands.parallel(
                        NamedCommands.getCommand("Shoot Command"),
                        Commands.sequence(
                            Commands.waitSeconds(1.5),
                            NamedCommands.getCommand("Wrist Wiggle Command")
                        )
                    ),
                    Commands.waitSeconds(4.5)
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftTrenchDoubleSweepPt2", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                
                Commands.parallel(
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command")
                )    
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }
    //Right Trench double sweep normal waiting to shoot after path 
    public static Command getRightTrenchDoubleSweep(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.race(
                    Commands.parallel(
                        AutoUtil.loadPathPlannerPath("RightTrenchDoubleSweepPt1", m_swerve, true),
                        NamedCommands.getCommand("IntakeWrist Down Command")
                    ),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    Commands.parallel(
                        NamedCommands.getCommand("Shoot Command"),
                        Commands.sequence(
                            Commands.waitSeconds(1.5),
                            NamedCommands.getCommand("Wrist Wiggle Command")
                        )
                    ),
                    Commands.waitSeconds(4.5)
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("RightTrenchDoubleSweepPt2", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                
                Commands.parallel(
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