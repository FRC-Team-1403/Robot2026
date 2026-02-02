package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.vision.Vision;

public class ShooterCommand extends Command {

    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    private final Vision m_vision;

    public ShooterCommand(Shooter m_shooter, ShooterHood m_shooterHood, Vision m_vision) {
        this.m_shooter = m_shooter;
        this.m_shooterHood = m_shooterHood;
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
                m_shooter.setTargetRPM(Constants.Shooter.veryFarRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.veryFarAngle);
            }
            else if (dist > Constants.Shooter.farDist) {
                m_shooter.setTargetRPM(Constants.Shooter.farRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.farAngle);
            }
            else if (dist > Constants.Shooter.kindaFarDist) {
                m_shooter.setTargetRPM(Constants.Shooter.kindaFarRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.kindaFarAngle);
            }
            else if (dist > Constants.Shooter.mediumDist) {
                m_shooter.setTargetRPM(Constants.Shooter.mediumRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.mediumAngle);
            }
            else if (dist > Constants.Shooter.kindaMediumDist) {
                m_shooter.setTargetRPM(Constants.Shooter.kindaMediumRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.kindaMediumAngle);
            }
            else if (dist > Constants.Shooter.closeDist) {
                m_shooter.setTargetRPM(Constants.Shooter.closeRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.closeAngle);
            }
            else if (dist > Constants.Shooter.kindaCloseDist) {
                m_shooter.setTargetRPM(Constants.Shooter.kindaCloseRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.kindaCloseAngle);
            }
            else {
                m_shooter.setTargetRPM(Constants.Shooter.superCloseRPM);
                m_shooterHood.setSetpoint(Constants.ShooterHood.superCloseAngle);
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