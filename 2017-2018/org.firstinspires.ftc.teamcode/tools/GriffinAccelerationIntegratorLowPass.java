package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

/**
 * Provides a naive implementation of an acceleration integration algorithm.
 * It does the basic physics and runs a low pass filter on the data
 * <p/>
 * Todo:
 * One you would actually want to use in a robot would, for example, likely
 * filter noise out of the acceleration data or more sophisticated processing.
 * look up what and how to filter noise,
 * look at better methods for approximating the integral, -> currently uses trapezoid rule, try instead simpson's rule
 */

public class GriffinAccelerationIntegratorLowPass implements BNO055IMU.AccelerationIntegrator {
    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    public static final double ACCELERATION_THRESHOLD = 0.1;
    String log;
    private BNO055IMU.Parameters parameters;
    private Position position;
    private Velocity velocity;
    private Acceleration acceleration;
    private Filter filterX;
    private Filter filterY;
    private Filter filterZ;

    public GriffinAccelerationIntegratorLowPass() {
        this.parameters = null;
        this.position = null;
        this.velocity = null;
        this.acceleration = null;
        this.filterX = null;
        this.filterY = null;
        this.filterZ = null;
    }

    @Override
    public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = initialPosition;
        this.velocity = initialVelocity;
        this.acceleration = null;

        this.log = "";
        this.filterX = new LowPassFilter();
        this.filterY = new LowPassFilter();
        this.filterZ = new LowPassFilter();
    }

    public Position getPosition() {
        return this.position;
    }

    public Velocity getVelocity() {
        return this.velocity;
    }

    public Acceleration getAcceleration() {
        return this.acceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0) {
            Acceleration previousAcceleration = acceleration;
            Velocity previousVelocity = velocity;

            acceleration = processRawAcceleration(linearAcceleration);

            // We can only integrate if we have a previous acceleration to baseline from,
            // we only want to integrate when the filters are at capacity
            if (acceleration != null && filterX.atCapacity() && filterY.atCapacity() && filterZ.atCapacity()) {

                if (previousAcceleration.acquisitionTime != 0) {
                    Velocity deltaVelocity = meanIntegrate(acceleration, previousAcceleration);
                    velocity = plus(velocity, deltaVelocity);
                }

                if (previousVelocity.acquisitionTime != 0) {
                    Position deltaPosition = meanIntegrate(velocity, previousVelocity);
                    position = plus(position, deltaPosition);
                }

                if (parameters.loggingEnabled) {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - previousAcceleration.acquisitionTime) * 1e-9, acceleration, velocity, position);
                    log += acceleration.acquisitionTime + ", " + acceleration.xAccel + ", " + acceleration.yAccel +
                            ", " + acceleration.zAccel + "\n";
                }
            } else {
                acceleration = linearAcceleration;
            }
        }
    }

    private Acceleration processRawAcceleration(Acceleration linearAcceleration) {
        linearAcceleration.xAccel = filterX.processValue(linearAcceleration.xAccel);
        linearAcceleration.yAccel = filterY.processValue(linearAcceleration.yAccel);
        linearAcceleration.zAccel = filterZ.processValue(linearAcceleration.zAccel);
        linearAcceleration = thresholdAcceleration(linearAcceleration);

        return linearAcceleration;
    }

    private Acceleration thresholdAcceleration(Acceleration acceleration) {
        if (acceleration.xAccel <= ACCELERATION_THRESHOLD && acceleration.xAccel >= -ACCELERATION_THRESHOLD)
            acceleration.xAccel = 0;
        if (acceleration.yAccel <= ACCELERATION_THRESHOLD && acceleration.yAccel >= -ACCELERATION_THRESHOLD)
            acceleration.yAccel = 0;
        if (acceleration.zAccel <= ACCELERATION_THRESHOLD && acceleration.zAccel >= -ACCELERATION_THRESHOLD)
            acceleration.zAccel = 0;

        return acceleration;
    }

    public String getLog() {
        return log;
    }
}