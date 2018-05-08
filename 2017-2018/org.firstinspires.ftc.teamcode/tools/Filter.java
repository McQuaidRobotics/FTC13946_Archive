package org.firstinspires.ftc.teamcode.tools;

/**
 * Created by David on 11/4/2016.
 * A filter class, all filters should extend this class
 */

public abstract class Filter {

    public abstract double processValue(double value);

    public abstract double getProcessedValue();

    public abstract boolean atCapacity();
}