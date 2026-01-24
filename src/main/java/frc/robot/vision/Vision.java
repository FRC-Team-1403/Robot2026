package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem that provides robot pose estimates from multiple cameras using AprilTags.
 */
public class Vision extends SubsystemBase {
    private final PhotonCamera m_cameraFront;
    private final PhotonCamera m_cameraBack;
    private final PhotonPoseEstimator m_poseEstimatorFront;
    private final PhotonPoseEstimator m_poseEstimatorBack;

    private Pose3d m_lastEstimatedPose;
    private double m_lastTimestamp;

    public Vision() {
        m_cameraFront = new PhotonCamera(Constants.Vision.kFrontCameraName);
        m_cameraBack = new PhotonCamera(Constants.Vision.kBackCameraName);

        m_poseEstimatorFront = new PhotonPoseEstimator(
                Constants.Vision.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.kFrontRobotToCamera);
        m_poseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_poseEstimatorBack = new PhotonPoseEstimator(
                Constants.Vision.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.kBackRobotToCamera);
        m_poseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_lastEstimatedPose = new Pose3d();
        m_lastTimestamp = 0.0;
    }

    public Pose3d getPose() {
        return m_lastEstimatedPose;
    }

    public Pose2d getPose2d() {
        return m_lastEstimatedPose.toPose2d();
    }

    public boolean hasPose() {
        return !m_lastEstimatedPose.equals(new Pose3d());
    }

    public double getTimestamp() {
        return m_lastTimestamp;
    }

    public boolean isFrontConnected() {
        return m_cameraFront.isConnected();
    }

    public boolean isBackConnected() {
        return m_cameraBack.isConnected();
    }

    @Override
    public void periodic() {
        boolean updated = false;

        // Process front camera
        for (var result : m_cameraFront.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = m_poseEstimatorFront.update(result);
            if (visionEst.isPresent()) {
                m_lastEstimatedPose = visionEst.get().estimatedPose;
                m_lastTimestamp = visionEst.get().timestampSeconds;
                updated = true;
            }
        }

        // Process back camera
        for (var result : m_cameraBack.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = m_poseEstimatorBack.update(result);
            if (visionEst.isPresent()) {
                // Simple logic: use the most recent timestamp
                if (visionEst.get().timestampSeconds > m_lastTimestamp) {
                    m_lastEstimatedPose = visionEst.get().estimatedPose;
                    m_lastTimestamp = visionEst.get().timestampSeconds;
                    updated = true;
                }
            }
        }

        // Logging
        Logger.recordOutput("Vision/FrontCameraConnected", m_cameraFront.isConnected());
        Logger.recordOutput("Vision/BackCameraConnected", m_cameraBack.isConnected());
        Logger.recordOutput("Vision/HasPose", hasPose());
        Logger.recordOutput("Vision/Timestamp", m_lastTimestamp);

        if (updated) {
            Pose2d pose2d = m_lastEstimatedPose.toPose2d();
            Logger.recordOutput("Vision/Pose2d", pose2d);
            Logger.recordOutput("Vision/X", pose2d.getX());
            Logger.recordOutput("Vision/Y", pose2d.getY());
            Logger.recordOutput("Vision/RotationDeg",
                    (pose2d.getRotation().getDegrees() + 360) % 360);
        }
    }
}
