package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class ShooterCommand extends Command {

    private final Shooter m_shooter;
    private final Vision m_vision;

    public ShooterCommand(Shooter m_shooter, Vision m_vision) {
        this.m_shooter = m_shooter;
        this.m_vision = m_vision;

        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        if (m_vision.hasPose()) {
            Pose2d robotPose = m_vision.getPose2d();

            // Calculate dist to goal
            double deltaX = Constants.Vision.kGoalX - robotPose.getX();
            double deltaY = Constants.Vision.kGoalY - robotPose.getY();

            double dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
            
            if (dist > Constants.Shooter.veryFarDist) {
                m_shooter.setRPM(Constants.Shooter.veryFarRPM);
            }
            else if (dist > Constants.Shooter.farDist) {
                m_shooter.setRPM(Constants.Shooter.farDist);
            }
            else if (dist > Constants.Shooter.kindaFarDist) {
                m_shooter.setRPM(Constants.Shooter.kindaFarRPM);
            }
            else if (dist > Constants.Shooter.mediumDist) {
                m_shooter.setRPM(Constants.Shooter.mediumDist);
            }
            else if (dist > Constants.Shooter.kindaMediumDist) {
                m_shooter.setRPM(Constants.Shooter.kindaMediumRPM);
            }
            else if (dist > Constants.Shooter.closeDist) {
                m_shooter.setRPM(Constants.Shooter.closeRPM);
            }
            else if (dist > Constants.Shooter.kindaCloseDist) {
                m_shooter.setRPM(Constants.Shooter.kindaCloseRPM);
            }
            else {
                m_shooter.setRPM(Constants.Shooter.superCloseDist);
            }
        }

        else {
            // cooked
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}