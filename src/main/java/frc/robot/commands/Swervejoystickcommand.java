package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveConstants;

/**
 * Teleop command - translates joystick inputs to swerve drive motion.
 * Applies deadband and slew rate limiting for smooth control.
 */
public class SwerveJoystickCommand extends Command {

    private final SwerveDrive m_swerveDrive;
    
    // Input suppliers
    private final DoubleSupplier m_xSupplier;          // Forward/backward
    private final DoubleSupplier m_ySupplier;          // Left/right
    private final DoubleSupplier m_rotSupplier;        // Rotation
    private final BooleanSupplier m_fieldRelativeSupplier;

    // Slew rate limiters (smooth acceleration)
    private final SlewRateLimiter m_xLimiter;
    private final SlewRateLimiter m_yLimiter;
    private final SlewRateLimiter m_rotLimiter;

    public SwerveJoystickCommand(
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

        // Limit acceleration to 3.0 units/second
        m_xLimiter = new SlewRateLimiter(3.0);
        m_yLimiter = new SlewRateLimiter(3.0);
        m_rotLimiter = new SlewRateLimiter(3.0);

        addRequirements(swerveDrive);
    }

    /**
     * Called every 20ms - processes inputs and drives robot.
     */
    @Override
    public void execute() {
        // Read joystick values
        double xSpeed = m_xSupplier.getAsDouble();
        double ySpeed = m_ySupplier.getAsDouble();
        double rotSpeed = m_rotSupplier.getAsDouble();
        
        // Apply deadband (ignore small movements < 10%)
        xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
        rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.1);

        // Apply slew rate limiting (smooth acceleration)
        xSpeed = m_xLimiter.calculate(xSpeed);
        ySpeed = m_yLimiter.calculate(ySpeed);
        rotSpeed = m_rotLimiter.calculate(rotSpeed);
        
        // Scale to actual velocities
        xSpeed = xSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
        ySpeed = ySpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
        rotSpeed = rotSpeed * SwerveConstants.kMaxAngularSpeed;

        // Drive the robot
        m_swerveDrive.drive(
            xSpeed,
            ySpeed,
            rotSpeed,
            m_fieldRelativeSupplier.getAsBoolean()
        );
    }

    /**
     * Stop robot when command ends.
     */
    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stop();
    }
    
    /**
     * This command never ends on its own - runs until interrupted.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}