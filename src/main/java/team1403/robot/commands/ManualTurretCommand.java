package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Turret;

public class ManualTurretCommand extends Command {
    private static final double kChangeDegrees = 0.75;

    private final Turret m_turret;
    private final BooleanSupplier m_leftBumper;
    private final BooleanSupplier m_rightBumper;

    public ManualTurretCommand(Turret turret, BooleanSupplier leftBumper, BooleanSupplier rightBumper) {
        m_turret = turret;
        m_leftBumper = leftBumper;
        m_rightBumper = rightBumper;
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentSetpoint = m_turret.getSetpoint();

        if (m_leftBumper.getAsBoolean()) {
            m_turret.setSetpoint(
                MathUtil.inputModulus(
                    currentSetpoint - kChangeDegrees,
                    Constants.Turret.kMinAngleDegrees,
                    Constants.Turret.kMaxAngleDegrees));
        } else if (m_rightBumper.getAsBoolean()) {
            m_turret.setSetpoint(
                MathUtil.inputModulus(
                    currentSetpoint + kChangeDegrees,
                    Constants.Turret.kMinAngleDegrees,
                    Constants.Turret.kMaxAngleDegrees));
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}