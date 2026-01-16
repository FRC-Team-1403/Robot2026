package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

public class Vision {
    private PhotonCamera m_camera;

    public Vision(String cameraName) {
        m_camera = new PhotonCamera(cameraName);
    }

    public Pose2d getPose() {
        var result = m_camera.getLatestResult();
        var target = result.getBestTarget();
        var transform = target.getBestCameraToTarget().plus(Constants.Vision.kCameraToRobotTransform);
        return new Pose2d(transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
    }

    public double getXPose() {
        return getPose().getX();
    }

    public double getYPose() {
        return getPose().getY();
    }
}