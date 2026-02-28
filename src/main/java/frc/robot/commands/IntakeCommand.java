package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;

public class IntakeCommand extends Command {
    private final Intake m_intake;
    //private final IntakeWrist m_intakeWrist;
    private final double power;

    public IntakeCommand(Intake m_intake, double power) {
        this.m_intake = m_intake;
        //this.m_intakeWrist = m_intakeWrist;
        this.power = power;

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        //m_intakeWrist.setSetpoint(Constants.IntakeWrist.kDeployedAngle);
    }

    @Override
    public void execute() {
        // if (m_intakeWrist.atSetpoint()) {
        //     m_intake.setIntakeRPM(Constants.Intake.RPM);
        // }
        m_intake.setIntakePower(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        //m_intakeWrist.setSetpoint(Constants.IntakeWrist.kStowedAngle);
    }
}