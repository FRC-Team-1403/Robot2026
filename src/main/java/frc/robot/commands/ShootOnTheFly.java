package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class ShootOnTheFly extends Command {

    private enum Zone {
        ALLIANCE, MIDDLE, OPPOSING
    }

    private final Shooter m_shooter;
    private final ShooterHood m_hood;
    private final Turret m_turret;
    private final Indexer m_indexer;
    private final Spindexer m_spindexer;
    private final Vision m_vision;
    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private final Supplier<Pose2d> m_poseSupplier;
    private final BooleanSupplier m_rightTrigger;
    private final BooleanSupplier m_leftTrigger;

    public ShootOnTheFly(
            Shooter shooter,
            ShooterHood hood,
            Turret turret,
            Indexer indexer,
            Spindexer spindexer,
            Vision vision,
            Supplier<ChassisSpeeds> speedsSupplier,
            Supplier<Pose2d> poseSupplier,
            BooleanSupplier rightTrigger,
            BooleanSupplier leftTrigger) {

        m_shooter = shooter;
        m_hood = hood;
        m_turret = turret;
        m_indexer = indexer;
        m_spindexer = spindexer;
        m_vision = vision;
        m_speedsSupplier = speedsSupplier;
        m_poseSupplier = poseSupplier;
        m_rightTrigger = rightTrigger;
        m_leftTrigger = leftTrigger;

        addRequirements(shooter, hood, turret, indexer, spindexer);
    }

    @Override
    public void initialize() {
        m_spindexer.stop();
        m_indexer.stop();
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_poseSupplier.get();
        ChassisSpeeds speeds = m_speedsSupplier.get();

        boolean isRed = DriverStation.getAlliance()
                .map(a -> a == Alliance.Red)
                .orElse(false);

        Zone zone = getZone(robotPose, isRed);
        Translation2d target = getTarget(zone, isRed);

        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        if (distance < 0.1) {
            m_spindexer.stop();
            m_indexer.stop();
            return;
        }

        double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = robotPose.getRotation().getDegrees();
        double turretAngleDeg = MathUtil.inputModulus(fieldAngleToGoal - robotHeading, -180, 180);
        m_turret.setSetpoint(turretAngleDeg);

        double staticSpeed = m_shooter.getWheelSpeedForDistance(distance);
        double staticPitch = m_shooter.getPitchRadiansForDistance(distance);
        double yaw = Math.atan2(deltaY, deltaX);

        double vx = staticSpeed * Math.cos(staticPitch) * Math.cos(yaw);
        double vy = staticSpeed * Math.cos(staticPitch) * Math.sin(yaw);
        double vz = staticSpeed * Math.sin(staticPitch);

        double vxComp = vx - speeds.vxMetersPerSecond;
        double vyComp = vy - speeds.vyMetersPerSecond;

        double shotSpeed = Math.sqrt(vxComp * vxComp + vyComp * vyComp + vz * vz);

        double targetPitchDeg = Math.toDegrees(Math.atan2(vz, Math.sqrt(vxComp * vxComp + vyComp * vyComp)));
        targetPitchDeg = MathUtil.clamp(targetPitchDeg,
                Constants.ShooterHood.kMinAngleDegrees,
                Constants.ShooterHood.kMaxAngleDegrees);
        m_hood.setSetpoint(targetPitchDeg);

        double targetRPM = m_shooter.getRPMForShotSpeed(shotSpeed);
        m_shooter.setFlywheelTargetRPM(targetRPM);

        double angleTolerance = Math.toDegrees(
                Math.atan2(Constants.ShootOnTheFly.kShotToleranceMeters, distance));

        boolean flywheelReady = m_shooter.isFlywheelAtSpeed();
        boolean turretReady = Math.abs(m_turret.getTurretAngle() - turretAngleDeg) < angleTolerance;
        boolean hoodReady = Math.abs(m_hood.getHoodAngle() - targetPitchDeg) < angleTolerance;
        boolean poseOk = m_vision.getPoseFOM() < Constants.ShootOnTheFly.kMaxPoseFOM;

        boolean autoShoot = zone == Zone.ALLIANCE
                && Constants.ShootOnTheFly.kAutoShootEnabled
                && flywheelReady
                && turretReady
                && hoodReady
                && poseOk;

        boolean triggerPressed = m_rightTrigger.getAsBoolean() || m_leftTrigger.getAsBoolean();
        boolean shoot = autoShoot || triggerPressed;

        if (shoot) {
            m_spindexer.setSpindexerRPM(Constants.ShootOnTheFly.kSpindexerFeedRPM);
            m_indexer.setIndexerRPM(Constants.ShootOnTheFly.kIndexerFeedRPM);
        } else {
            m_spindexer.stop();
            m_indexer.stop();
        }

        Logger.recordOutput("ShootOnTheFly/RobotPose", robotPose);
        Logger.recordOutput("ShootOnTheFly/TargetPose", new Pose2d(target, new Rotation2d()));
        Logger.recordOutput("ShootOnTheFly/Zone", zone.toString());
        Logger.recordOutput("ShootOnTheFly/IsRed", isRed);
        Logger.recordOutput("ShootOnTheFly/Distance", distance);
        Logger.recordOutput("ShootOnTheFly/TurretSetpointDeg", turretAngleDeg);
        Logger.recordOutput("ShootOnTheFly/HoodSetpointDeg", targetPitchDeg);
        Logger.recordOutput("ShootOnTheFly/TargetRPM", targetRPM);
        Logger.recordOutput("ShootOnTheFly/AngleToleranceDeg", angleTolerance);
        Logger.recordOutput("ShootOnTheFly/FlywheelReady", flywheelReady);
        Logger.recordOutput("ShootOnTheFly/TurretReady", turretReady);
        Logger.recordOutput("ShootOnTheFly/HoodReady", hoodReady);
        Logger.recordOutput("ShootOnTheFly/PoseOk", poseOk);
        Logger.recordOutput("ShootOnTheFly/AutoShoot", autoShoot);
        Logger.recordOutput("ShootOnTheFly/RightTrigger", m_rightTrigger.getAsBoolean());
        Logger.recordOutput("ShootOnTheFly/LeftTrigger", m_leftTrigger.getAsBoolean());
        Logger.recordOutput("ShootOnTheFly/Shooting", shoot);
        Logger.recordOutput("ShootOnTheFly/PoseFOM", m_vision.getPoseFOM());
        Logger.recordOutput("ShootOnTheFly/RobotVX", speeds.vxMetersPerSecond);
        Logger.recordOutput("ShootOnTheFly/RobotVY", speeds.vyMetersPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        m_spindexer.stop();
        m_indexer.stop();
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private Zone getZone(Pose2d robotPose, boolean isRed) {
        double x = robotPose.getX();
        if (isRed) {
            if (x >= Constants.ShootOnTheFly.kRedAllianceZoneMinX) {
                return Zone.ALLIANCE;
            } else if (x <= Constants.ShootOnTheFly.kBlueAllianceZoneMaxX) {
                return Zone.OPPOSING;
            } else {
                return Zone.MIDDLE;
            }
        } else {
            if (x <= Constants.ShootOnTheFly.kBlueAllianceZoneMaxX) {
                return Zone.ALLIANCE;
            } else if (x >= Constants.ShootOnTheFly.kRedAllianceZoneMinX) {
                return Zone.OPPOSING;
            } else {
                return Zone.MIDDLE;
            }
        }
    }

    private Translation2d getTarget(Zone zone, boolean isRed) {
        if (zone == Zone.ALLIANCE) {
            if (isRed) {
                return Constants.ShootOnTheFly.kRedHubLocation;
            } else {
                return Constants.ShootOnTheFly.kBlueHubLocation;
            }
        }

        if (zone == Zone.MIDDLE || zone == Zone.OPPOSING) {
            if (isRed) {
                if (m_rightTrigger.getAsBoolean()) {
                    return Constants.ShootOnTheFly.kRedFeedLocationRight;
                } else {
                    return Constants.ShootOnTheFly.kRedFeedLocationLeft;
                }
            } else {
                if (m_rightTrigger.getAsBoolean()) {
                    return Constants.ShootOnTheFly.kBlueFeedLocationRight;
                } else {
                    return Constants.ShootOnTheFly.kBlueFeedLocationLeft;
                }
            }
        }

        return Constants.ShootOnTheFly.kBlueHubLocation;
    }
}