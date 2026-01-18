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
 * Vision subsystem that provides robot pose estimates from AprilTag detection.
 * This class continuously updates the robot's field-relative position based on
 * visible AprilTags.
 */
public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poseEstimator;
    private Pose3d m_lastEstimatedPose;
    private double m_lastTimestamp;

    public Vision() {
        m_camera = new PhotonCamera(Constants.Vision.kCameraName);
        
        m_poseEstimator = new PhotonPoseEstimator(
            Constants.Vision.kFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.Vision.kRobotToCamera
        );
        
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        m_lastEstimatedPose = new Pose3d();
        m_lastTimestamp = 0.0;
    }

    /**
     * Gets the latest robot pose estimate from vision in 3D.
     * 
     * @return The robot's field-relative Pose3d, or a zero pose if no estimate is available
     */
    public Pose3d getPose() {
        return m_lastEstimatedPose;
    }

    /**
     * Gets the latest robot pose estimate from vision in 2D.
     * 
     * @return The robot's field-relative Pose2d, or a zero pose if no estimate is available
     */
    public Pose2d getPose2d() {
        return m_lastEstimatedPose.toPose2d();
    }

    /**
     * Checks if vision currently has a valid pose estimate.
     * 
     * @return true if we've received at least one pose estimate from vision
     */
    public boolean hasPose() {
        return !m_lastEstimatedPose.equals(new Pose3d());
    }

    /**
     * Gets the timestamp of the last pose estimate.
     * 
     * @return The timestamp in seconds, or 0.0 if no estimate is available
     */
    public double getTimestamp() {
        return m_lastTimestamp;
    }

    /**
     * Checks if the camera is currently connected.
     * 
     * @return true if the camera is connected
     */
    public boolean isConnected() {
        return m_camera.isConnected();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Pose2d", getPose2d());
        Logger.recordOutput("Pose2d X", getPose2d().getX());
        Logger.recordOutput("Pose2d Y", getPose2d().getY());
        Logger.recordOutput("Pose2d Rotation", (getPose2d().getRotation().getDegrees()+360)%360);

        // Process all unread camera results
        for (var result : m_camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = m_poseEstimator.update(result);
            
            if (visionEst.isPresent()) {
                m_lastEstimatedPose = visionEst.get().estimatedPose;
                m_lastTimestamp = visionEst.get().timestampSeconds;
            }
        }    
    }
}
