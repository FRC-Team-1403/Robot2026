package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonFX m_intakeMotor;
  private final VelocityVoltage m_intakeVelocityRequest;
  private final DutyCycleOut m_intakelDutyCycleRequest;
  private double m_intakeTargetRPM = 0;
  private double m_intakeTargetDutyCycle = 0;
  private boolean m_intakeUseVelocityControl = true;
  private final StatusSignal<AngularVelocity> m_intakeVelocity;

  public Intake() {
    m_intakeMotor = new TalonFX(Constants.Intake.m_intakeID,"Bus 2");

    m_intakeVelocityRequest = new VelocityVoltage(0);
    m_intakeVelocityRequest.Slot = 0;
    m_intakeVelocityRequest.EnableFOC = false;
    m_intakelDutyCycleRequest = new DutyCycleOut(0);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 120;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 70;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    Slot0Configs intakePIDConfig = new Slot0Configs();
    intakePIDConfig.kP = Constants.Intake.kP; 
    intakePIDConfig.kI = Constants.Intake.kI;
    intakePIDConfig.kD = Constants.Intake.kD; 
    intakePIDConfig.kS = Constants.Intake.kS; 
    intakePIDConfig.kV = Constants.Intake.kV; 
    intakePIDConfig.kA = Constants.Intake.kA; 
    intakeConfig.Slot0 = intakePIDConfig;

    m_intakeMotor.getConfigurator().apply(intakeConfig);

    m_intakeVelocity = m_intakeMotor.getVelocity();
  }

  public void setIntakeRPM(double rpm) {
    m_intakeTargetRPM = rpm;
    m_intakeVelocityRequest.Velocity = rpm * Constants.Intake.intakeGearRatio / 60.0;
    m_intakeUseVelocityControl = true;
  }

  public void setIntakePower(double dutyCycle) {
    m_intakeTargetDutyCycle = dutyCycle;
    m_intakelDutyCycleRequest.Output = dutyCycle;
    m_intakeUseVelocityControl = false;
  }

  public void stop() {
    setIntakeRPM(0);
  }

  public double getIntakeRPM() {
    return m_intakeVelocity.getValueAsDouble() * 60.0 / Constants.Intake.intakeGearRatio;
  }

  public double getIntakeTargetRPM() {
    return m_intakeTargetRPM;
  }

  public double getIntakeRPMError() {
    return m_intakeTargetRPM - getIntakeRPM();
  }

  public boolean isIntakeAtSpeed() {
    return Math.abs(getIntakeRPMError()) < Constants.Intake.rpmTolerance;
  }

  public double getIntakeTargetDutyCycle() {
    return m_intakeTargetDutyCycle;
  }

  @Override
  public void periodic() {
    m_intakeVelocity.refresh();

    if (m_intakeUseVelocityControl) {
      m_intakeMotor.setControl(m_intakeVelocityRequest);
    } else {
      m_intakeMotor.setControl(m_intakelDutyCycleRequest);
    }

    Logger.recordOutput("Intake/Target RPM", m_intakeTargetRPM);
    Logger.recordOutput("Intake/Leader RPM", getIntakeRPM());
    Logger.recordOutput("Intake/RPM Error", getIntakeRPMError());
    Logger.recordOutput("Intake/At Speed", isIntakeAtSpeed());
    Logger.recordOutput("Intake/Target Duty Cycle", m_intakeTargetDutyCycle);
    Logger.recordOutput("Intake/Leader Voltage", m_intakeMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Intake/Leader Stator Current", m_intakeMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/Supply Current", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/Torque Current", m_intakeMotor.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Intake/Closed Loop Error", m_intakeMotor.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput(
        "Intake/Closed Loop Output", m_intakeMotor.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput("Intake/Duty Cycle", m_intakeMotor.getDutyCycle().getValueAsDouble() * 1000);
    Logger.recordOutput("Intake/Leader Temp", m_intakeMotor.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Intake/Using Velocity Control", m_intakeUseVelocityControl);
  }
}
