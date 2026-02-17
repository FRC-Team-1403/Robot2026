package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.GroundIntake;

public class GroundIntakeCommandPower extends Command {
    private final GroundIntake m_groundIntake;
    private final double m_intakeTargetPower;

    public GroundIntakeCommandPower(GroundIntake groundIntake, double intakeTargetPower) {
        m_groundIntake = groundIntake;
        m_intakeTargetPower = intakeTargetPower;
        addRequirements(groundIntake);
    }

    @Override
    public void initialize() {
        m_groundIntake.setIntakePower(m_intakeTargetPower);
    }

    @Override
    public void execute() {
        m_groundIntake.setIntakePower(m_intakeTargetPower);
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