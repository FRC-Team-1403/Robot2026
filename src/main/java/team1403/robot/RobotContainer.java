package team1403.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.commands.ShootOnMoveCommand;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Turret;
import team1403.robot.swerve.FakeSwerveDrive;
import team1403.robot.util.ShooterTable;

import java.io.IOException;

public class RobotContainer {
  private final FakeSwerveDrive m_swerve      = new FakeSwerveDrive();
  private final Shooter         m_shooter     = new Shooter();
  private final ShooterHood     m_shooterHood = new ShooterHood();
  private final Turret          m_turret      = new Turret();

  private ShootOnMoveCommand m_sotmCommand = null;

  public RobotContainer() {
    ShooterTable table = null;
    try {
      table = ShooterTable.load();
      System.out.println("[ShooterTable] Loaded successfully.");
    } catch (IOException e) {
      DriverStation.reportError("[ShooterTable] Failed to load: " + e.getMessage(), false);
    }

    if (table != null) {
      m_sotmCommand = new ShootOnMoveCommand(
          m_shooter, m_shooterHood, m_turret,
          m_swerve::getPose, m_swerve::getChassisSpeeds, table);
    }
  }

  /** Called from Robot.teleopInit() — schedules the command when the robot is actually enabled. */
  public void onTeleopInit() {
    if (m_sotmCommand != null) {
      CommandScheduler.getInstance().schedule(m_sotmCommand);
    }
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
