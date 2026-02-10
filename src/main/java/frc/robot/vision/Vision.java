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

    private static class WeightedPose {
        public final Pose3d pose;
        public final double weight;

        public WeightedPose(Pose3d pose, double ambiguity) {
            this.pose = pose;
            this.weight = 1.0 / (ambiguity + 0.01);
        }
    }

    public static class CameraFrustum {
        public final Pose3d pose;
        public final double horizontalFOV;
        public final double verticalFOV;
        public final double range;

        public CameraFrustum(Pose3d pose, double horizontalFOV, double verticalFOV, double range) {
            this.pose = pose;
            this.horizontalFOV = horizontalFOV;
            this.verticalFOV = verticalFOV;
            this.range = range;
        }
    }

    public Vision() {
        m_camera1 = new PhotonCamera(Constants.Vision.kCamera1);
        m_camera2 = new PhotonCamera(Constants.Vision.kCamera2);
        m_camera3 = new PhotonCamera(Constants.Vision.kCamera3);
        m_camera4 = new PhotonCamera(Constants.Vision.kCamera4);

        m_visionSim = new VisionSystemSim("main");
        m_visionSim.addAprilTags(Constants.Vision.kFieldLayout);

        SimCameraProperties camProps1 = new SimCameraProperties();
        camProps1.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        camProps1.setFPS(30);
        m_cameraSim1 = new PhotonCameraSim(m_camera1, camProps1);
        m_visionSim.addCamera(m_cameraSim1, Constants.Vision.kCamera1Transform);

        SimCameraProperties camProps2 = new SimCameraProperties();
        camProps2.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        camProps2.setFPS(30);
        m_cameraSim2 = new PhotonCameraSim(m_camera2, camProps2);
        m_visionSim.addCamera(m_cameraSim2, Constants.Vision.kCamera2Transform);

        SimCameraProperties camProps3 = new SimCameraProperties();
        camProps3.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        camProps3.setFPS(30);
        m_cameraSim3 = new PhotonCameraSim(m_camera3, camProps3);
        m_visionSim.addCamera(m_cameraSim3, Constants.Vision.kCamera3Transform);

        SimCameraProperties camProps4 = new SimCameraProperties();
        camProps4.setCalibration(640, 480, Rotation2d.fromDegrees(70));
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

    private Pose3d combinePoses(List<WeightedPose> weightedPoses) {
        if (weightedPoses.isEmpty()) {
            return m_robotPose;
        }

        double totalWeight = weightedPoses.stream()
                .mapToDouble(wp -> wp.weight)
                .sum();

        double avgX = weightedPoses.stream()
                .mapToDouble(wp -> wp.pose.getX() * wp.weight)
                .sum() / totalWeight;

        double avgY = weightedPoses.stream()
                .mapToDouble(wp -> wp.pose.getY() * wp.weight)
                .sum() / totalWeight;

        double avgZ = weightedPoses.stream()
                .mapToDouble(wp -> wp.pose.getZ() * wp.weight)
                .sum() / totalWeight;

        double sinSum = 0.0;
        double cosSum = 0.0;

        for (WeightedPose wp : weightedPoses) {
            double angleRad = wp.pose.getRotation().toRotation2d().getRadians();
            sinSum += Math.sin(angleRad) * wp.weight;
            cosSum += Math.cos(angleRad) * wp.weight;
        }

        double avgAngleRad = Math.atan2(sinSum, cosSum);

        return new Pose3d(
                avgX, avgY, avgZ,
                new Rotation3d(0, 0, avgAngleRad));
    }

    private void logCameraData(PhotonPipelineResult result, String cameraName, Transform3d cameraTransform) {
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

            Pose3d cameraPose = m_robotPose.transformBy(cameraTransform);
            Logger.recordOutput("Vision/" + cameraName + "/CameraPose", cameraPose);

            if (result.getMultiTagResult().isPresent()) {
                Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
                Pose3d estimatedRobotPose = new Pose3d().transformBy(fieldToCamera).transformBy(cameraTransform.inverse());
                Logger.recordOutput("Vision/" + cameraName + "/EstimatedRobotPose", estimatedRobotPose);
            }
        } else {
            Logger.recordOutput("Vision/" + cameraName + "/TagCount", 0);
            Logger.recordOutput("Vision/" + cameraName + "/VisibleTagIDs", new int[0]);
            Logger.recordOutput("Vision/" + cameraName + "/TagPoses", new Pose3d[0]);
        }
    }

    @Override
    public void periodic() {
        List<WeightedPose> weightedPoses = new ArrayList<>();

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

        CameraFrustum frustum1 = new CameraFrustum(cameraPose1, Math.toRadians(70), Math.toRadians(50), 5.0);
        CameraFrustum frustum2 = new CameraFrustum(cameraPose2, Math.toRadians(70), Math.toRadians(50), 5.0);
        CameraFrustum frustum3 = new CameraFrustum(cameraPose3, Math.toRadians(70), Math.toRadians(50), 5.0);
        CameraFrustum frustum4 = new CameraFrustum(cameraPose4, Math.toRadians(70), Math.toRadians(50), 5.0);

        Logger.recordOutput("Vision/Camera1Frustum", new Pose3d[] {
            frustum1.pose,
            new Pose3d(
                frustum1.pose.getX() + frustum1.range * Math.cos(frustum1.pose.getRotation().getZ()),
                frustum1.pose.getY() + frustum1.range * Math.sin(frustum1.pose.getRotation().getZ()),
                frustum1.pose.getZ(),
                frustum1.pose.getRotation()
            )
        });

        Logger.recordOutput("Vision/Camera2Frustum", new Pose3d[] {
            frustum2.pose,
            new Pose3d(
                frustum2.pose.getX() + frustum2.range * Math.cos(frustum2.pose.getRotation().getZ()),
                frustum2.pose.getY() + frustum2.range * Math.sin(frustum2.pose.getRotation().getZ()),
                frustum2.pose.getZ(),
                frustum2.pose.getRotation()
            )
        });

        Logger.recordOutput("Vision/Camera3Frustum", new Pose3d[] {
            frustum3.pose,
            new Pose3d(
                frustum3.pose.getX() + frustum3.range * Math.cos(frustum3.pose.getRotation().getZ()),
                frustum3.pose.getY() + frustum3.range * Math.sin(frustum3.pose.getRotation().getZ()),
                frustum3.pose.getZ(),
                frustum3.pose.getRotation()
            )
        });

        Logger.recordOutput("Vision/Camera4Frustum", new Pose3d[] {
            frustum4.pose,
            new Pose3d(
                frustum4.pose.getX() + frustum4.range * Math.cos(frustum4.pose.getRotation().getZ()),
                frustum4.pose.getY() + frustum4.range * Math.sin(frustum4.pose.getRotation().getZ()),
                frustum4.pose.getZ(),
                frustum4.pose.getRotation()
            )
        });

        Pose2d pose2d = m_robotPose.toPose2d();
        Logger.recordOutput("Vision/RobotPose", pose2d);
        Logger.recordOutput("Vision/X", pose2d.getX());
        Logger.recordOutput("Vision/Y", pose2d.getY());
        Logger.recordOutput("Vision/RotationDeg", (pose2d.getRotation().getDegrees() + 360) % 360);
    }
}