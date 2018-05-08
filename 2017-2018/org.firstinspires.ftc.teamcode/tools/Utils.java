package org.firstinspires.ftc.teamcode.tools;

/**
 * Created by Jonathan on 11/28/2017.
 */

public class Utils {
    public static double clamp (double min, double max, double val) {
        if (val > max) return max;
        else if (val < min) return min;
        else return val;
    }

    public static boolean inRange(double min, double max, double val) {
        if (val >= min && val <= max) return true;
        else return false;
    }

    public static double encoderToDistance(int pos) {
        double distanceForOneRotation = (1.5) * (4 * Math.PI);
        double distanceForOneEncoderTick = distanceForOneRotation/1440.0;
        return distanceForOneEncoderTick * pos;
    }

    public static double rgbToHue(double r, double g, double b) {
        r /= 255;
        g /= 255;
        b /= 255;

        double max = Math.max(Math.max(r,g),b);
        double min = Math.min(Math.min(r,g),b);

        double hue = -1;
        if (r == max) {
            hue = (g-b)/(max-min);
        } else if (g == max) {
            hue = 2.0 + (b-r)/(max-min);
        } else if (b == max) {
            hue = 4.0 + (r-g)/(max-min);
        }
        hue *= 60;
        if (hue < 0) hue += 360;

        return hue;
    }

    public static boolean isGemRed(double r, double g, double b) {
        double hue = rgbToHue(r, g, b);
        if (inRange(0,20,hue) || inRange(340, 360, hue)) return true;
        else return false;
    }

    public static boolean isGemBlue(double r, double g, double b) {
        double hue = rgbToHue(r, g, b);
        if (inRange(165,305,hue)) return true;
        else return false;
    }
}
