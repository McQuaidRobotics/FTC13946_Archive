package org.firstinspires.ftc.teamcode.tools;

/**
 * Created by David on 10/27/2016.
 * Implements a low pass filter
 * on doubles
 */
public class LowPassFilter extends Filter {

    //private ArrayList<Double> values;
    private double processedValue;
    private boolean isInitialized;

    public LowPassFilter() {
        //values = new ArrayList<>();
        processedValue = 0;
        isInitialized = false;
    }

    public double processValue(double newValue) {
        //values.add(newValue);
        /*if (values.size() > capacity) {
            double removed = values.remove(0);
            //for (int i = 0; i < values.size(); i++) {
            double calc = prev[i] + alpha * (input[i] - prev[i]);
            //}
            *//*processedValue -= removed / capacity;
            processedValue += newValue / capacity;*//*
        } else {
            double sum = 0;
            for (double value : values) {
                sum += value;
            }

            processedValue = sum / values.size();*/
        //}
        if (isInitialized) {
            processedValue = processedValue + 0.03 * (newValue - processedValue);
        } else {
            processedValue = newValue;
            isInitialized = true;
        }
        return processedValue;
    }

    public double getProcessedValue() {
        return processedValue;
    }

    public boolean atCapacity() {
        return isInitialized;
    }
}