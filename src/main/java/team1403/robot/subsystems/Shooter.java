package team1403.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team1403.robot.Constants;
import team1403.robot.Robot;

public class Shooter extends SubsystemBase {
  private final boolean m_isSim = Robot.isSimulation();

  // Real hardware — only used on real robot
  private TalonFX m_flywheelLeader;
  private TalonFX m_flywheelFollower;
  private TalonFX m_flywheelFollower2;
  private VelocityVoltage m_flywheelVelocityRequest;
  private DutyCycleOut m_flywheelDutyCycleRequest;
  @SuppressWarnings("all") private StatusSignal m_leaderVel;
  @SuppressWarnings("all") private StatusSignal m_followerVel;
  @SuppressWarnings("all") private StatusSignal m_follower2Vel;

  private double  m_targetRPM       = 0;
  private boolean m_velocityControl = true;

  public Shooter() {
    if (!m_isSim) {
      m_flywheelLeader    = new TalonFX(Constants.Shooter.flywheelLeaderID, "Bus 2");
      m_flywheelFollower  = new TalonFX(Constants.Shooter.flywheelFollower1TopRightID, "Bus 2");
      m_flywheelFollower2 = new TalonFX(Constants.Shooter.flywheelFollower2BottonRightID, "Bus 2");
      m_flywheelVelocityRequest = new VelocityVoltage(0);
      m_flywheelVelocityRequest.Slot = 0;
      m_flywheelVelocityRequest.EnableFOC = true;
      m_flywheelDutyCycleRequest = new DutyCycleOut(0);

      TalonFXConfiguration leaderCfg = new TalonFXConfiguration();
      leaderCfg.MotorOutput.Inverted   = InvertedValue.CounterClockwise_Positive;
      leaderCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      leaderCfg.CurrentLimits.StatorCurrentLimit       = 120;
      leaderCfg.CurrentLimits.StatorCurrentLimitEnable = true;
      leaderCfg.CurrentLimits.SupplyCurrentLimit       = 70;
      leaderCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
      leaderCfg.CurrentLimits.SupplyCurrentLowerLimit  = 40;
      leaderCfg.CurrentLimits.SupplyCurrentLowerTime   = 1.0;
      Slot0Configs pid = new Slot0Configs();
      pid.kP = Constants.Shooter.kP; pid.kI = Constants.Shooter.kI; pid.kD = Constants.Shooter.kD;
      pid.kS = Constants.Shooter.kS; pid.kV = Constants.Shooter.kV; pid.kA = Constants.Shooter.kA;
      leaderCfg.Slot0 = pid;
      m_flywheelLeader.getConfigurator().apply(leaderCfg);

      TalonFXConfiguration followerCfg = new TalonFXConfiguration();
      followerCfg.MotorOutput.Inverted   = InvertedValue.Clockwise_Positive;
      followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      m_flywheelFollower.getConfigurator().apply(followerCfg);
      m_flywheelFollower.setControl(
          new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));
      m_flywheelFollower2.getConfigurator().apply(followerCfg);
      m_flywheelFollower2.setControl(
          new Follower(Constants.Shooter.flywheelLeaderID, MotorAlignmentValue.Opposed));

      m_leaderVel    = m_flywheelLeader.getVelocity();
      m_followerVel  = m_flywheelFollower.getVelocity();
      m_follower2Vel = m_flywheelFollower2.getVelocity();
    }
  }

  public void setFlywheelTargetRPM(double rpm) {
    m_targetRPM = rpm;
    m_velocityControl = true;
    if (!m_isSim) {
      m_flywheelVelocityRequest.Velocity = (rpm * Constants.Shooter.flywheelGearRatio) / 60.0;
    }
  }

  public void stop() { setFlywheelTargetRPM(0); }

  /** In sim, reports the target RPM directly (instant "at speed" for visualization). */
  public double getFlywheelLeaderRPM() {
    if (m_isSim) return m_targetRPM;
    return (m_leaderVel.getValueAsDouble() * 60.0) / Constants.Shooter.flywheelGearRatio;
  }

  public double getFlywheelTargetRPM() { return m_targetRPM; }
  public double getFlywheelRPMError()  { return m_targetRPM - getFlywheelLeaderRPM(); }
  public boolean isFlywheelAtSpeed()   {
    return Math.abs(getFlywheelRPMError()) < Constants.Shooter.rpmTolerance;
  }

  @Override
  public void periodic() {
    if (!m_isSim) {
      m_leaderVel.refresh(); m_followerVel.refresh(); m_follower2Vel.refresh();
      if (m_velocityControl) m_flywheelLeader.setControl(m_flywheelVelocityRequest);
      else                   m_flywheelLeader.setControl(m_flywheelDutyCycleRequest);
    }
    Logger.recordOutput("Flywheel/Target RPM", m_targetRPM);
    Logger.recordOutput("Flywheel/Leader RPM", getFlywheelLeaderRPM());
    Logger.recordOutput("Flywheel/At Speed",   isFlywheelAtSpeed());
  }
}
