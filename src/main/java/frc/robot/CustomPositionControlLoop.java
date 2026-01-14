package frc.robot;

public class CustomPositionControlLoop {

    private double gain;
    private double positionWindow;
    private double rampUpTime;
    private double rampDownTime;
    private double unitsPerRampTime;
    private double maxSpeed;
    private double minSpeed;
    private double progRate;

    private double rampOutput = 0.0;

    private double p = 0.0;
    private double output = 0.0;
    private boolean atPosition = false;

    public CustomPositionControlLoop(double gain, double positionWindow, double rampUpTime, double rampDownTime,
                                    double unitsPerRampTime, double maxSpeed, double minSpeed, double progRate) {
        this.gain = gain;
        this.positionWindow = positionWindow;
        this.rampUpTime = rampUpTime;
        this.rampDownTime = rampDownTime;
        this.unitsPerRampTime = unitsPerRampTime;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.progRate = progRate;
    }

    private boolean ramp(double input) {
        boolean rampDone;

        if (input > rampOutput) {
            rampOutput = rampOutput + (unitsPerRampTime / (rampUpTime / progRate));
            
            if (rampOutput > input) {
                rampOutput = input;
            }
        }

        if (input < rampOutput) {
            rampOutput = rampOutput - (unitsPerRampTime / (rampDownTime / progRate));
            
            if (rampOutput < input) {
                rampOutput = input;
            }
            
        }

        if (Math.abs(input - rampOutput) < 0.001) {
            rampDone = true;
        } else {
            rampDone = false;
        }

        return rampDone;
    }

    public double calculate(double setpoint, double feedback) {

        double error;
        boolean negativeFlag;
        double outputSetpoint;
        boolean rampDone;

        error = setpoint - feedback;

        p = error * gain;

        if (p < 0) {
            negativeFlag = true;
        } else {
            negativeFlag = false;
        }

        outputSetpoint = Math.abs(p);

        if (outputSetpoint > maxSpeed) {
            outputSetpoint = maxSpeed;
        }

        rampDone = ramp(outputSetpoint);

        output = rampOutput;

        if (rampDone && output < minSpeed) {
            output = minSpeed;
        }

        if (negativeFlag) {
            output = output * -1;
        }

        if (feedback > (setpoint - positionWindow) && feedback < (setpoint + positionWindow)) {
            output = 0;
            atPosition = true;
        } else {
            atPosition = false;
        }

        return output;
    }

    public boolean isAtPosition() {
        return atPosition;
    }

    public double getP() {
        return p;
    }

    public double getOutput() {
        return output;
    }

    public void reset() {
        rampOutput = 0.0;
    }
}