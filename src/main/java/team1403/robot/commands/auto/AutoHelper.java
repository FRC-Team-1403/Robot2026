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

    public static Command getStationaryCenterShootAutoAlign(SwerveSubsystem m_swerve){
        try{
           return Commands.sequence(
            NamedCommands.getCommand("IntakeWrist Down Command"),
            AutoUtil.loadPathPlannerPath("StationaryHub", m_swerve, true),
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

    public static Command getMiddleHubDepotEndHub(SwerveSubsystem m_swerve){
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
                AutoUtil.loadPathPlannerPath("MiddleHubDepotPt3Hub", m_swerve, true),
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

    public static Command getLeftTrenchDoubleSweep(SwerveSubsystem m_swerve) {
    try {
        return Commands.sequence(
            Commands.race(
                Commands.parallel(
                    NamedCommands.getCommand("IntakeWrist Down Command"),
                    AutoUtil.loadPathPlannerPath("LeftTrenchSweepPt1", m_swerve, true),
                    Commands.sequence(
                        Commands.waitSeconds(6.0),
                        Commands.parallel(
                            NamedCommands.getCommand("Shoot Command"),
                            Commands.waitSeconds(2.5),
                            NamedCommands.getCommand("Wrist Wiggle Command")
                        )
                    )
                ),
                NamedCommands.getCommand("Intake Command")
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

public static Command getLeftTrenchDoubleSweepDepot(SwerveSubsystem m_swerve) {
    try {
        return Commands.sequence(
            Commands.race(
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("LeftTrenchSweepDepotPt1", m_swerve, true),
                    NamedCommands.getCommand("IntakeWrist Down Command")
                ),
                NamedCommands.getCommand("Intake Command")
            ),
            Commands.race(
                NamedCommands.getCommand("Shoot Command"),
                NamedCommands.getCommand("Wrist Wiggle Command"),
                Commands.waitSeconds(3)
            ),
            Commands.race(
                AutoUtil.loadPathPlannerPath("LeftTrenchSweepDepotPt2", m_swerve, true),
                NamedCommands.getCommand("Intake Command")
            ),
            Commands.race(
                AutoUtil.loadPathPlannerPath("LeftTrenchSweepDepotPt3", m_swerve, true),
                NamedCommands.getCommand("Shoot Command"),
                NamedCommands.getCommand("Intake Command"),
                Commands.waitSeconds(7.0)
            )
        );
    } catch (Exception e) {
        System.err.println("Could not load auto: " + e.getMessage());
        return Commands.none();
    }
}

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

    public static Command getLeftBumpDoubleSweepDepot(SwerveSubsystem m_swerve) {
        try {    
            return Commands.sequence(
                Commands.race(
                    Commands.parallel(
                        AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepDepotPt1", m_swerve, true),
                        NamedCommands.getCommand("IntakeWrist Down Command")
                    ),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepDepotPt2", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepDepotPt3", m_swerve, true),
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command")
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepDepotPt4", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepDepotPt5", m_swerve, true),
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Wrist Wiggle Command")                    
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepDepotPt6", m_swerve, true),
                    NamedCommands.getCommand("Shoot Command"),
                    NamedCommands.getCommand("Intake Command")                    
                )
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }


    public static Command getLeftBumpDoubleSweepMiddle(SwerveSubsystem m_swerve) {
        try {    
            return Commands.sequence(
                Commands.race(
                    Commands.parallel(
                        AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepMiddlePt1", m_swerve, true),
                        NamedCommands.getCommand("IntakeWrist Down Command")
                    ),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepMiddlePt2", m_swerve, true),
                    NamedCommands.getCommand("Intake Command")
                ),
                Commands.waitSeconds(0.5),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepMiddlePt3", m_swerve, true),
                    NamedCommands.getCommand("Shoot Command"),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        NamedCommands.getCommand("Wrist Wiggle Command")
                    ),
                    Commands.waitSeconds(6)

                ),
                Commands.race(
                    AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepMiddlePt4", m_swerve, true),
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
                    Commands.waitSeconds(5.0)
                ),
                AutoUtil.loadPathPlannerPath("LeftBumpDoubleSweepMiddlePt5", m_swerve, true)

            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

    public static Command ChoreoTesting(SwerveSubsystem m_swerve) {
            try {
                return Commands.sequence(
                    // Start sweep + intake down + intake 
                    Commands.deadline(
                        AutoUtil.loadChoreoPath("Sweep1", m_swerve),
                        Commands.parallel(
                            NamedCommands.getCommand("IntakeWrist Down Command"),
                            NamedCommands.getCommand("Intake Command")
                        )
                    ),
                    // Shoot
                    Commands.deadline(
                        Commands.waitSeconds(Constants.Autos.TestingShootTimeout),
                        NamedCommands.getCommand("Shoot Command"),
                        NamedCommands.getCommand("Wrist Wiggle Command")
                    ),
                    // Sweep 2 + Intake
                    Commands.deadline(
                        AutoUtil.loadChoreoPath("Sweep3", m_swerve),
                        NamedCommands.getCommand("Intake Command")
                    ),
                    // Shoot
                    Commands.deadline(
                        Commands.waitSeconds(Constants.Autos.TestingShootTimeout),
                        NamedCommands.getCommand("Shoot Command"),
                        NamedCommands.getCommand("Wrist Wiggle Command")
                    ),
                    // Sweep 3 :skull:
                    Commands.deadline(
                        AutoUtil.loadChoreoPath("Sweep3", m_swerve),
                        NamedCommands.getCommand("Intake Command"),
                        NamedCommands.getCommand("Shoot Command") //Not sure if u want passing here
                    )
                );
            }
            catch (Exception e) {
                System.err.println("Could not load auto: " + e.getMessage());
                return Commands.none();
            }
        }
}