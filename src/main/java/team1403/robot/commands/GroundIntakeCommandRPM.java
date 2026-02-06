package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.GroundIntake;

public class GroundIntakeCommandRPM extends Command {
    private final GroundIntake m_groundIntake;
    private final double m_intakeTargetRPM;

    public GroundIntakeCommandRPM(GroundIntake groundIntake, double intakeTargetRPM) {
        m_groundIntake = groundIntake;
        m_intakeTargetRPM = intakeTargetRPM;
        addRequirements(groundIntake);
    }

    @Override
    public void initialize() {
        m_groundIntake.setIntakeTargetRPM(m_intakeTargetRPM);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_groundIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}