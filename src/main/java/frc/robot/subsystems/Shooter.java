package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Shooter extends SubsystemBase {
  private final SparkMax m_motor1;
  private final SparkMax m_motor2;
  private final RelativeEncoder m_encoder;
  private SimpleMotorFeedforward m_feedForward;
  private ProfiledPIDController m_profiled;
  private double m_targetRPM = 0;
  private double m_targetDutyCycle = 0;
  private boolean m_useVelocityControl = false;

  public Shooter() {
    m_motor1 = new SparkMax(3, MotorType.kBrushless);
    m_motor2 = new SparkMax(2, MotorType.kBrushless);
    
    SparkMaxConfig config1 = new SparkMaxConfig();
    config1.idleMode(IdleMode.kCoast);
    config1.smartCurrentLimit(40);
    
    SparkMaxConfig config2 = new SparkMaxConfig();
    config2.idleMode(IdleMode.kCoast);
    config2.smartCurrentLimit(40);
    config2.follow(3, true);
    
    m_motor1.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_motor2.configure(config2, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    m_encoder = m_motor1.getEncoder();
    
    m_feedForward = new SimpleMotorFeedforward(
      Constants.Shooter.kS,
      Constants.Shooter.kV
    );
    
    m_profiled = new ProfiledPIDController(
      Constants.Shooter.kP,
      Constants.Shooter.kI,
      Constants.Shooter.kD,
      new TrapezoidProfile.Constraints(
        Constants.Shooter.maxVelocityRPM,
        Constants.Shooter.maxAccelerationRPMPerSec
      )
    );
    
    m_profiled.setTolerance(Constants.Shooter.rpmTolerance);
  }
  
  public void setTargetRPM(double rpm) {
    m_targetRPM = rpm;
    m_useVelocityControl = rpm > 0;
  }
  
  public void setTargetPower(double dutyCycle) {
    m_targetDutyCycle = dutyCycle;
    m_useVelocityControl = false;
  }
  
  public void stop() {
    m_targetRPM = 0;
    m_targetDutyCycle = 0;
    m_useVelocityControl = false;
    m_motor1.set(0);
  }
  
  public double getRPM() {
    return m_encoder.getVelocity();
  }
  
  public double getTargetRPM() {
    return m_targetRPM;
  }
  
  public double getRPMError() {
    return m_targetRPM - getRPM();
  }
  
  public boolean isAtSpeed() {
    return m_profiled.atGoal();
  }
  
  public double getTargetDutyCycle() {
    return m_targetDutyCycle;
  }

  @Override
  public void periodic() {
    if (m_useVelocityControl && m_targetRPM > 0) {
      double pidOutput = m_profiled.calculate(getRPM(), m_targetRPM);
      double ffOutput = m_feedForward.calculate(m_targetRPM / 60.0);
      m_motor1.setVoltage(pidOutput + ffOutput);
    } else {
      m_motor1.set(m_targetDutyCycle);
    }
    
    SmartDashboard.putNumber("Shooter/Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter/Actual RPM", getRPM());
    SmartDashboard.putNumber("Shooter/RPM Error", getRPMError());
    SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
    SmartDashboard.putNumber("Shooter/Target Duty Cycle", m_targetDutyCycle);
    SmartDashboard.putNumber("Shooter/Motor Voltage", m_motor1.getBusVoltage() * m_motor1.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Motor Current", m_motor1.getOutputCurrent());
    SmartDashboard.putNumber("Shooter/Duty Cycle", m_motor1.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Motor Temp", m_motor1.getMotorTemperature());
    SmartDashboard.putBoolean("Shooter/Using Velocity Control", m_useVelocityControl);
    SmartDashboard.putNumber("Shooter/Setpoint RPM", m_profiled.getSetpoint().velocity);
  }
}
