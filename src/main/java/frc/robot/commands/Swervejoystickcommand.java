package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveConstants;

public class Swervejoystickcommand extends Command {

    private final SwerveDrive m_swerveDrive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_rotSupplier;
    private final BooleanSupplier m_fieldRelativeSupplier;

    private final SlewRateLimiter m_xLimiter;
    private final SlewRateLimiter m_yLimiter;
    private final SlewRateLimiter m_rotLimiter;

    public Swervejoystickcommand(
        SwerveDrive swerveDrive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier rotSupplier,
        BooleanSupplier fieldRelativeSupplier
    ) {

        m_swerveDrive = swerveDrive;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_rotSupplier = rotSupplier;
        m_fieldRelativeSupplier = fieldRelativeSupplier;

        m_xLimiter = new SlewRateLimiter(3.0);
        m_yLimiter = new SlewRateLimiter(3.0);
        m_rotLimiter = new SlewRateLimiter(3.0);

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(m_xSupplier.getAsDouble(), 0.1);
        double ySpeed = MathUtil.applyDeadband(m_ySupplier.getAsDouble(), 0.1);
        double rotSpeed = MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), 0.1);

        xSpeed = m_xLimiter.calculate(xSpeed) * SwerveConstants.kMaxSpeedMetersPerSecond;
        ySpeed = m_yLimiter.calculate(ySpeed) * SwerveConstants.kMaxSpeedMetersPerSecond;
        rotSpeed = m_rotLimiter.calculate(rotSpeed) * SwerveConstants.kMaxAngularSpeed;

        m_swerveDrive.drive(
            xSpeed,
            ySpeed,
            rotSpeed,
            m_fieldRelativeSupplier.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stop();
    }
}
