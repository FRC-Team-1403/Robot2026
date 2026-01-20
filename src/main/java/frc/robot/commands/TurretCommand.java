package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class TurretCommand extends Command {

    private final Turret m_turret;
    private final Vision m_vision;

    public TurretCommand(Turret m_turret, Vision m_vision) {
        this.m_turret = m_turret;
        this.m_vision = m_vision;

        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_vision.hasPose()) {
            double deltaX = Constants.Vision.kGoalX - m_vision.getPose2d().getX();
            double deltaY = Constants.Vision.kGoalY - m_vision.getPose2d().getY();
            double fieldAngleToGoal = Math.atan2(deltaY, deltaX) * (180.0 / Math.PI);

            double robotHeading = m_vision.getPose2d().getRotation().getDegrees();
            double turretAngle = ((fieldAngleToGoal - robotHeading + 1080)%360)-180;

            SmartDashboard.putNumber("Testing/Calculated Angle", turretAngle);
            SmartDashboard.putNumber("Testing/Delta X", deltaX);
            SmartDashboard.putNumber("Testing/Delta Y", deltaY);
            SmartDashboard.putNumber("Testing/Field Angle to Goal", fieldAngleToGoal);
            SmartDashboard.putNumber("Testing/Robot Heading (CCW)", robotHeading);
            SmartDashboard.putNumber("Testing/Robot X", m_vision.getPose2d().getX());
            SmartDashboard.putNumber("Testing/Robot Y", m_vision.getPose2d().getY());
            SmartDashboard.putNumber("Testing/Robot Heading CCW", m_vision.getPose2d().getRotation().getDegrees());
            Logger.recordOutput("Pose2d", m_vision.getPose2d());
            Logger.recordOutput("Goal", new Pose2d(Constants.Vision.kGoalX, Constants.Vision.kGoalY, new Rotation2d()));
            Logger.recordOutput("Angle", turretAngle);

            m_turret.setSetpoint(turretAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}