package org.firstinspires.ftc.teamcode.tools;

/**
 * Created by Jonathan on 12/19/2017.
 */

public class AveragingFilter {
    private double rampSpeed;
    private double value;
    public AveragingFilter(double rampSpeed) {
        this.rampSpeed = Utils.clamp(0, 1, rampSpeed);
        value = 0;
    }
    public double filter(double val) {
        //value = value + rampSpeed * (val - value);
        value = (1 - rampSpeed) * value + (rampSpeed) * val;
        if (Math.abs(value) < rampSpeed) return 0;
        //if (val == 0 && Utils.inRange(-rampSpeed, rampSpeed, value)) value = 0;
        return value;
    }
    public double getVal() {
        return value;
    }
    public void setRampSpeed(double rampSpeed) {
        this.rampSpeed = rampSpeed;
    }
}
