package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera1;
    private final PhotonCamera m_camera2;
    private final PhotonCamera m_camera3;
    private final PhotonCamera m_camera4;

    private final VisionSystemSim m_visionSim;
    private final PhotonCameraSim m_cameraSim1;
    private final PhotonCameraSim m_cameraSim2;
    private final PhotonCameraSim m_cameraSim3;
    private final PhotonCameraSim m_cameraSim4;

    private Pose3d m_robotPose;

    public Vision() {
        m_camera1 = new PhotonCamera(Constants.Vision.kCamera1);
        m_camera2 = new PhotonCamera(Constants.Vision.kCamera2);
        m_camera3 = new PhotonCamera(Constants.Vision.kCamera3);
        m_camera4 = new PhotonCamera(Constants.Vision.kCamera4);

        m_visionSim = new VisionSystemSim("main");
        m_visionSim.addAprilTags(Constants.Vision.kFieldLayout);

        SimCameraProperties camProps1 = new SimCameraProperties();
        camProps1.setCalibration(640, 480, Rotation2d.fromDegrees(80));  // 80° lens for ThriftyCam
        camProps1.setFPS(30);
        m_cameraSim1 = new PhotonCameraSim(m_camera1, camProps1);
        m_visionSim.addCamera(m_cameraSim1, Constants.Vision.kCamera1Transform);

        SimCameraProperties camProps2 = new SimCameraProperties();
        camProps2.setCalibration(640, 480, Rotation2d.fromDegrees(80));
        camProps2.setFPS(30);
        m_cameraSim2 = new PhotonCameraSim(m_camera2, camProps2);
        m_visionSim.addCamera(m_cameraSim2, Constants.Vision.kCamera2Transform);

        SimCameraProperties camProps3 = new SimCameraProperties();
        camProps3.setCalibration(640, 480, Rotation2d.fromDegrees(80));
        camProps3.setFPS(30);
        m_cameraSim3 = new PhotonCameraSim(m_camera3, camProps3);
        m_visionSim.addCamera(m_cameraSim3, Constants.Vision.kCamera3Transform);

        SimCameraProperties camProps4 = new SimCameraProperties();
        camProps4.setCalibration(640, 480, Rotation2d.fromDegrees(80));  // 80° lens for ThriftyCamera
        camProps4.setFPS(30);
        m_cameraSim4 = new PhotonCameraSim(m_camera4, camProps4);
        m_visionSim.addCamera(m_cameraSim4, Constants.Vision.kCamera4Transform);

        m_robotPose = new Pose3d();
    }

    public Pose3d getPose() {
        return m_robotPose;
    }

    public Pose2d getPose2d() {
        return m_robotPose.toPose2d();
    }

    public void setSimPose(Pose2d pose) {
        m_robotPose = new Pose3d(pose);
        m_visionSim.update(m_robotPose);
    }

    public void updatePose(Pose2d pose) {
        m_robotPose = new Pose3d(pose);
    }

    /**
     * Creates a simple camera direction indicator
     * Returns array of 2 poses: [camera position, point 5m in front of camera]
     */
    private Pose3d[] createCameraDirectionLine(Pose3d cameraPose, double range) {
        double yaw = cameraPose.getRotation().getZ();
        
        Pose3d endpoint = new Pose3d(
            cameraPose.getX() + range * Math.cos(yaw),
            cameraPose.getY() + range * Math.sin(yaw),
            cameraPose.getZ(),
            cameraPose.getRotation()
        );
        
        return new Pose3d[] { cameraPose, endpoint };
    }

    /**
     * Creates a wider FOV cone using 5 lines to show the camera's field of view
     */
    private Pose3d[] createCameraFOVCone(Pose3d cameraPose, double horizontalFOV, double range) {
        double yaw = cameraPose.getRotation().getZ();
        double halfFOV = horizontalFOV / 2.0;
        
        Pose3d center = new Pose3d(
            cameraPose.getX() + range * Math.cos(yaw),
            cameraPose.getY() + range * Math.sin(yaw),
            cameraPose.getZ(),
            cameraPose.getRotation()
        );
        
        Pose3d leftEdge = new Pose3d(
            cameraPose.getX() + range * Math.cos(yaw + halfFOV),
            cameraPose.getY() + range * Math.sin(yaw + halfFOV),
            cameraPose.getZ(),
            cameraPose.getRotation()
        );
        
        Pose3d rightEdge = new Pose3d(
            cameraPose.getX() + range * Math.cos(yaw - halfFOV),
            cameraPose.getY() + range * Math.sin(yaw - halfFOV),
            cameraPose.getZ(),
            cameraPose.getRotation()
        );
        
        return new Pose3d[] {
            cameraPose,
            leftEdge,
            cameraPose,
            center,
            cameraPose,
            rightEdge
        };
    }

    private void logCameraData(PhotonPipelineResult result, String cameraName, Transform3d cameraTransform) {
        Pose3d cameraPose = m_robotPose.transformBy(cameraTransform);
        
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            double ambiguity = bestTarget.getPoseAmbiguity();

            Logger.recordOutput("Vision/" + cameraName + "/TagCount", targets.size());
            Logger.recordOutput("Vision/" + cameraName + "/Ambiguity", ambiguity);

            int[] visibleTagIDs = new int[targets.size()];
            Pose3d[] tagPoses = new Pose3d[targets.size()];

            for (int i = 0; i < targets.size(); i++) {
                PhotonTrackedTarget target = targets.get(i);
                int fiducialId = target.getFiducialId();
                visibleTagIDs[i] = fiducialId;

                Optional<Pose3d> tagPose = Constants.Vision.kFieldLayout.getTagPose(fiducialId);
                if (tagPose.isPresent()) {
                    tagPoses[i] = tagPose.get();
                } else {
                    tagPoses[i] = new Pose3d();
                }
            }

            Logger.recordOutput("Vision/" + cameraName + "/VisibleTagIDs", visibleTagIDs);
            Logger.recordOutput("Vision/" + cameraName + "/TagPoses", tagPoses);
            Logger.recordOutput("Vision/" + cameraName + "/CameraPose", cameraPose);

            // Create lines from camera to each detected tag
            List<Pose3d> detectionLines = new ArrayList<>();
            for (Pose3d tagPose : tagPoses) {
                if (tagPose.getX() != 0 || tagPose.getY() != 0) {  // Skip empty poses
                    detectionLines.add(cameraPose);
                    detectionLines.add(tagPose);
                }
            }
            Logger.recordOutput("Vision/" + cameraName + "/DetectionLines", detectionLines.toArray(new Pose3d[0]));

            if (result.getMultiTagResult().isPresent()) {
                Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
                Pose3d estimatedRobotPose = new Pose3d().transformBy(fieldToCamera).transformBy(cameraTransform.inverse());
                Logger.recordOutput("Vision/" + cameraName + "/EstimatedRobotPose", estimatedRobotPose);
            }
        } else {
            Logger.recordOutput("Vision/" + cameraName + "/TagCount", 0);
            Logger.recordOutput("Vision/" + cameraName + "/VisibleTagIDs", new int[0]);
            Logger.recordOutput("Vision/" + cameraName + "/TagPoses", new Pose3d[0]);
            Logger.recordOutput("Vision/" + cameraName + "/DetectionLines", new Pose3d[0]);
        }
    }

    @Override
    public void periodic() {
        var results1 = m_camera1.getAllUnreadResults();
        if (!results1.isEmpty()) {
            PhotonPipelineResult result1 = results1.get(results1.size() - 1);
            logCameraData(result1, "Camera1", Constants.Vision.kCamera1Transform);
        }

        var results2 = m_camera2.getAllUnreadResults();
        if (!results2.isEmpty()) {
            PhotonPipelineResult result2 = results2.get(results2.size() - 1);
            logCameraData(result2, "Camera2", Constants.Vision.kCamera2Transform);
        }

        var results3 = m_camera3.getAllUnreadResults();
        if (!results3.isEmpty()) {
            PhotonPipelineResult result3 = results3.get(results3.size() - 1);
            logCameraData(result3, "Camera3", Constants.Vision.kCamera3Transform);
        }

        var results4 = m_camera4.getAllUnreadResults();
        if (!results4.isEmpty()) {
            PhotonPipelineResult result4 = results4.get(results4.size() - 1);
            logCameraData(result4, "Camera4", Constants.Vision.kCamera4Transform);
        }

        Pose3d cameraPose1 = m_robotPose.transformBy(Constants.Vision.kCamera1Transform);
        Pose3d cameraPose2 = m_robotPose.transformBy(Constants.Vision.kCamera2Transform);
        Pose3d cameraPose3 = m_robotPose.transformBy(Constants.Vision.kCamera3Transform);
        Pose3d cameraPose4 = m_robotPose.transformBy(Constants.Vision.kCamera4Transform);

        Logger.recordOutput("Vision/Camera1Direction", createCameraDirectionLine(cameraPose1,1));
        Logger.recordOutput("Vision/Camera2Direction", createCameraDirectionLine(cameraPose2, 1));
        Logger.recordOutput("Vision/Camera3Direction", createCameraDirectionLine(cameraPose3, 1));
        Logger.recordOutput("Vision/Camera4Direction", createCameraDirectionLine(cameraPose4, 1));

        Logger.recordOutput("Vision/Camera1FOV", createCameraFOVCone(cameraPose1, Math.toRadians(80),1));  
        Logger.recordOutput("Vision/Camera2FOV", createCameraFOVCone(cameraPose2, Math.toRadians(80), 1));
        Logger.recordOutput("Vision/Camera3FOV", createCameraFOVCone(cameraPose3, Math.toRadians(80), 1));
        Logger.recordOutput("Vision/Camera4FOV", createCameraFOVCone(cameraPose4, Math.toRadians(80), 1));  

        Logger.recordOutput("Vision/CameraPoses", new Pose3d[] {
            cameraPose1, cameraPose2, cameraPose3, cameraPose4
        });

        Pose2d pose2d = m_robotPose.toPose2d();
        Logger.recordOutput("Vision/RobotPose", pose2d);
        Logger.recordOutput("Vision/RobotPose3d", m_robotPose);
        Logger.recordOutput("Vision/X", pose2d.getX());
        Logger.recordOutput("Vision/Y", pose2d.getY());
        Logger.recordOutput("Vision/RotationDeg", (pose2d.getRotation().getDegrees() + 360) % 360);
    }
}