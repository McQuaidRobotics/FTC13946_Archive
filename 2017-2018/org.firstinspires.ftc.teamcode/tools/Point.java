package org.firstinspires.ftc.teamcode.tools;

//import java.lang.Math.*;

/**
 * Created by Jonathan on 11/28/2017.
 */

public class Point {
    private double x, y;
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public static double distance(Point currentPoint, Point nextPoint)
    {
        return Math.sqrt(Math.pow(currentPoint.x() - nextPoint.x(), 2) + Math.pow(currentPoint.y() - nextPoint.y(), 2));

    }

    public static double getAngle(Point currentPoint, Point nextPoint)
    {
        if (Math.abs(currentPoint.x() - nextPoint.x()) < 0.5) {
            return 0;
        } else {
            double slope = (currentPoint.y() - nextPoint.y()) / (currentPoint.x() - nextPoint.x());
            double angle = Math.toDegrees(Math.atan(slope));
            return angle;
            /*if (angle < 0.1) return 0;
            else return Math.toDegrees(Math.atan(slope));*/
        }
    }

    public static Point getCenter(double distanceBetweenPoints, double turnAngle, Point currentPoint)
    {
        double triAngle = 90 - Math.abs(turnAngle);
        double radius =  (distanceBetweenPoints / 2) / Math.cos(Math.toRadians(triAngle));
        Point center = new Point(currentPoint.x + radius, currentPoint.y);
        return center;
    }

    public static double getArcDistance(double Radius, double turnAngle) //Point currentPoint, Point nextPoint, Point center
    {
        double arcAngle = 2 * Math.abs(turnAngle); //Point.getAngle(center, currentPoint) - Point.getAngle(center, nextPoint ) ;
        System.out.println("arcAngle" + arcAngle);
        return (arcAngle / 360) * 2 * Math.PI * Radius;
    }

    public String toString() {
        return "p( " + this.x + " , " + this.y + " )";
    }
}
