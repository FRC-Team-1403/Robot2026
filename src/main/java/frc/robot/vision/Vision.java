package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera1;
    private final PhotonCamera m_camera2;
    private final PhotonCamera m_camera3;

    private final PhotonPoseEstimator m_poseEstimator1;
    private final PhotonPoseEstimator m_poseEstimator2;
    private final PhotonPoseEstimator m_poseEstimator3;

    private Pose3d m_combinedPose;
    private double m_lastTimestamp;

    private static class WeightedPose {
        public final Pose3d pose;
        public final double weight;

        public WeightedPose(Pose3d pose, double ambiguity) {
            this.pose = pose;
            this.weight = 1.0 / (ambiguity + 0.01);
        }
    }

    public Vision() {
        m_camera1 = new PhotonCamera(Constants.Vision.kCamera1);
        m_camera2 = new PhotonCamera(Constants.Vision.kCamera2);
        m_camera3 = new PhotonCamera(Constants.Vision.kCamera3);

        m_poseEstimator1 = new PhotonPoseEstimator(
                Constants.Vision.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.kCamera1Transform);
        m_poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_poseEstimator2 = new PhotonPoseEstimator(
                Constants.Vision.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.kCamera2Transform);
        m_poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_poseEstimator3 = new PhotonPoseEstimator(
                Constants.Vision.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.kCamera3Transform);
        m_poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_combinedPose = new Pose3d();
        m_lastTimestamp = 0.0;
    }

    public Pose3d getPose() {
        return m_combinedPose;
    }

    public Pose2d getPose2d() {
        return m_combinedPose.toPose2d();
    }

    public boolean hasPose() {
        return !m_combinedPose.equals(new Pose3d());
    }

    public double getTimestamp() {
        return m_lastTimestamp;
    }

    public boolean isCamera1Connected() {
        return m_camera1.isConnected();
    }

    public boolean isCamera2Connected() {
        return m_camera2.isConnected();
    }

    public boolean isCamera3Connected() {
        return m_camera3.isConnected();
    }

    private Pose3d combinePoses(List<WeightedPose> weightedPoses) {
        if (weightedPoses.isEmpty()) {
            return m_combinedPose;
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

    @Override
    public void periodic() {
        List<WeightedPose> weightedPoses = new ArrayList<>();
        double latestTimestamp = m_lastTimestamp;

        for (var result : m_camera1.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = m_poseEstimator1.update(result);
            if (visionEst.isPresent() && result.hasTargets()) {
                double ambiguity = result.getBestTarget().getPoseAmbiguity();
                weightedPoses.add(new WeightedPose(visionEst.get().estimatedPose, ambiguity));
                latestTimestamp = Math.max(latestTimestamp, visionEst.get().timestampSeconds);
                Logger.recordOutput("Vision/Camera1/Pose", visionEst.get().estimatedPose.toPose2d());
                Logger.recordOutput("Vision/Camera1/TagCount", result.getTargets().size());
                Logger.recordOutput("Vision/Camera1/Ambiguity", ambiguity);
            }
        }

        for (var result : m_camera2.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = m_poseEstimator2.update(result);
            if (visionEst.isPresent() && result.hasTargets()) {
                double ambiguity = result.getBestTarget().getPoseAmbiguity();
                weightedPoses.add(new WeightedPose(visionEst.get().estimatedPose, ambiguity));
                latestTimestamp = Math.max(latestTimestamp, visionEst.get().timestampSeconds);
                Logger.recordOutput("Vision/Camera2/Pose", visionEst.get().estimatedPose.toPose2d());
                Logger.recordOutput("Vision/Camera2/TagCount", result.getTargets().size());
                Logger.recordOutput("Vision/Camera2/Ambiguity", ambiguity);
            }
        }

        for (var result : m_camera3.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = m_poseEstimator3.update(result);
            if (visionEst.isPresent() && result.hasTargets()) {
                double ambiguity = result.getBestTarget().getPoseAmbiguity();
                weightedPoses.add(new WeightedPose(visionEst.get().estimatedPose, ambiguity));
                latestTimestamp = Math.max(latestTimestamp, visionEst.get().timestampSeconds);
                Logger.recordOutput("Vision/Camera3/Pose", visionEst.get().estimatedPose.toPose2d());
                Logger.recordOutput("Vision/Camera3/TagCount", result.getTargets().size());
                Logger.recordOutput("Vision/Camera3/Ambiguity", ambiguity);
            }
        }

        if (!weightedPoses.isEmpty()) {
            m_combinedPose = combinePoses(weightedPoses);
            m_lastTimestamp = latestTimestamp;
        }

        Logger.recordOutput("Vision/Camera1Connected", m_camera1.isConnected());
        Logger.recordOutput("Vision/Camera2Connected", m_camera2.isConnected());
        Logger.recordOutput("Vision/Camera3Connected", m_camera3.isConnected());
        Logger.recordOutput("Vision/ActiveCameraCount", weightedPoses.size());
        Logger.recordOutput("Vision/HasPose", hasPose());
        Logger.recordOutput("Vision/Timestamp", m_lastTimestamp);
        Pose2d pose2d = m_combinedPose.toPose2d();
        Logger.recordOutput("Vision/CombinedPose", pose2d);
        Logger.recordOutput("Vision/X", pose2d.getX());
        Logger.recordOutput("Vision/Y", pose2d.getY());
        Logger.recordOutput("Vision/RotationDeg",
                (pose2d.getRotation().getDegrees() + 360) % 360);
    }
}