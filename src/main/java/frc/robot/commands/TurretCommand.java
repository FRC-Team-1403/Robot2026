package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Vision;

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

            double robotHeading = (m_vision.getPose2d().getRotation().getDegrees() + 360) % 360;
            double turretAngle = ((fieldAngleToGoal - robotHeading) + 1080) % 360;

            SmartDashboard.putNumber("Calculated Angle", turretAngle);

            m_turret.setSetpoint(turretAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return m_turret.atSetpoint();
    }
}