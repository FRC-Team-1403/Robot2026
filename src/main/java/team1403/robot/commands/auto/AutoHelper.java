// package team1403.robot.commands.auto;

// import com.pathplanner.lib.auto.AutoBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import team1403.robot.util.AutoUtil;
// import team1403.robot.util.CougarUtil;
// import team1403.robot.swerve.SwerveSubsystem;
// import team1403.robot.swerve.TunerConstants;

// public class AutoHelper {

//     public static Command alignToStartingPose(SwerveSubsystem m_swerve, String pathName) {
//         return m_swerve.defer(() -> {
//             Pose2d target = AutoUtil.getStartingPose(pathName);
//             if (target == null) return Commands.none();
//             return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
//         });
//     }

//     public static Command getMoveAuto(SwerveSubsystem m_swerve) {
//         return m_swerve.defer(() -> {
//             Pose2d target = m_swerve.getPose();
//             if(CougarUtil.getAlliance() == Alliance.Red)
//                 target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.kZero, 1);
//             else
//                 target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.k180deg, 1);
//             return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
//         });
//     }

//         public static Command Depot(SwerveSubsystem m_swerve) {
//         try {
//             return Commands.sequence(
//                 //Add in the shoot command
//                 AutoUtil.loadPathPlannerPath("DepotPt1", m_swerve, true), //fix names tmr
//                 Commands.waitSeconds(10) //pick up balls from depot and shoot from their at the same thing
//                 //later add in the parellel squence to intake and shoot at the same time at the depot
//             );
//         } catch (Exception e) {
//             System.err.println("Could not load auto: " + e.getMessage());
//             return Commands.none();
//         }
//     }

//     public static Command LeftSweep(SwerveSubsystem m_swerve) {
//         try {
//             return Commands.sequence(
//                 AutoUtil.loadPathPlannerPath("LeftSweepPt1", m_swerve, true), //fix names tmr
//                 AutoUtil.loadPathPlannerPath("LeftSweepPt2", m_swerve, true), 
//                 Commands.waitSeconds(5),
//                 //Add in the shoot command
//                 AutoUtil.loadPathPlannerPath("LeftSweepPt3", m_swerve, true),
//                 AutoUtil.loadPathPlannerPath("LeftSweepPt4", m_swerve, true),
//                 Commands.waitSeconds(5)
//                 //Add in the shoot command
//             );
//         } catch (Exception e) {
//             System.err.println("Could not load auto: " + e.getMessage());
//             return Commands.none();
//         }
//     }

//     public static Command RightSweep(SwerveSubsystem m_swerve) {
//         try {
//             return Commands.sequence(
//                 AutoUtil.loadPathPlannerPath("RightSweepPt1", m_swerve, true), //fix names tmr
//                 AutoUtil.loadPathPlannerPath("RightSweepPt2", m_swerve, true), 
//                 Commands.waitSeconds(5),
//                 //Add in the shoot command
//                 AutoUtil.loadPathPlannerPath("RightSweepPt3", m_swerve, true),
//                 AutoUtil.loadPathPlannerPath("RightSweepPt4", m_swerve, true),
//                 Commands.waitSeconds(5)
//                 //Add in the shoot command
//             );
//         } catch (Exception e) {
//             System.err.println("Could not load auto: " + e.getMessage());
//             return Commands.none();
//         }
//     }

//     public static Command centerDepot(SwerveSubsystem m_swerve) {
//         try {
//             return Commands.sequence(
//                 AutoUtil.loadPathPlannerPath("Center+DepotPt1", m_swerve, true), //fix names tmr
//                 AutoUtil.loadPathPlannerPath("Center+DepotPt2", m_swerve, true), 
//                 Commands.waitSeconds(5)
//                 //Add in the shoot command
//                 //later add in the parellel squence to intake and shoot at the same time at the depot
//             );
//         } catch (Exception e) {
//             System.err.println("Could not load auto: " + e.getMessage());
//             return Commands.none();
//         }
//     }

//     public static Command centerHuman(SwerveSubsystem m_swerve) {
//         try {
//             return Commands.sequence(
//                 AutoUtil.loadPathPlannerPath("Center+HumanPt1", m_swerve, true), //fix names tmr
//                 AutoUtil.loadPathPlannerPath("Center+HumanPt2", m_swerve, true), 
//                 Commands.waitSeconds(5)
//                 //Add in the shoot command
//                 //later add in the parellel squence to intake and shoot at the same time at the human player station
//             );
//         } catch (Exception e) {
//             System.err.println("Could not load auto: " + e.getMessage());
//             return Commands.none();
//         }
//     }

// }
