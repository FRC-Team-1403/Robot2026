package frc.robot.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TurretEncoder {
    
    private DutyCycleEncoder encoder;
    private CustomPositionControlLoop controller;
    private final double deadzoneAngle = 15;
    private final double deadzoneSize = 115;
    private double currentAngle;
    private double setpoint;
    
    public TurretEncoder(int port) {
        encoder = new DutyCycleEncoder(port);
        
        controller = new CustomPositionControlLoop(
            Constants.TurretConstants.kGain,
            Constants.TurretConstants.kToleranceDegrees,
            Constants.TurretConstants.kRampUpTime,
            Constants.TurretConstants.kRampDownTime,
            100.0,
            Constants.TurretConstants.kMaxSpeed,
            Constants.TurretConstants.kMinSpeed,
            Constants.TurretConstants.kLoopTime
        );
        
        currentAngle = getTurretAngle();
        setpoint = currentAngle;
    }
    
    public double getTurretAngle() {
        double angle = encoder.get();
        if(angle > 360) {
            angle -= 360;
        }
        angle = angle * Constants.TurretConstants.gearRatio * 360;
        return angle;
    }
    
    public void setSetpoint(double degrees) {
        setpoint = degrees % 360.0;
        if(setpoint < 0) {
            setpoint += 360.0;
        }
    }
    
    public double getSetpoint() {
        return setpoint;
    }
    
    public double getRaw() {
        return encoder.get();
    }
    
    public double getRawWithGearRatio() {
        return encoder.get() * Constants.TurretConstants.gearRatio;
    }
    
    public double getDeadzoneWidth() {
        return this.deadzoneSize;
    }
    
    public double getDeadzoneAngle() {
        return this.deadzoneAngle;
    }
    
    public double getControllerOutput() {
        currentAngle = getTurretAngle();
        double error = getShortestRotation(setpoint, currentAngle);
        return controller.calculate(error, currentAngle, setpoint);
    }
    
    private double getShortestRotation(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }
        return error;
    }
    
    public boolean atSetpoint() {
        return controller.isAtPosition();
    }
    
    public double getP() {
        return controller.getP();
    }
    
    public void reset() {
        controller.reset();
    }
    
    public boolean isConnected() {
        return encoder.isConnected();
    }
}