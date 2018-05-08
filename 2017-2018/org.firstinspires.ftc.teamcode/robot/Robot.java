package org.firstinspires.ftc.teamcode.robot;

import android.content.Intent;
//import android.media.MediaPlayer;
//import android.provider.MediaStore;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.tools.AveragingFilter;
import org.firstinspires.ftc.teamcode.tools.GriffinAccelerationIntegratorLowPass;
import org.firstinspires.ftc.teamcode.tools.Point;
import org.firstinspires.ftc.teamcode.tools.Utils;
import org.firstinspires.ftc.teamcode.tools.Path;


/**
 * Created by Jonathan on 11/28/2017.
 */
public class Robot {
    public static final int CLOSE_THRESHOLD = 10;
    public static final int DRIVE_ENCODER_TICKS = 1120;
    public static final int ELEVATOR_ENCODER_TICKS = 1440;
    public static final int BALL_CHECK_DURATION = 500;
    public static final double ROBOT_WIDTH = 17.0;
    public static final int WHEEL_DIAMETER = 4;
    public static final int BLUE_TEAM = 0;
    public static final int RED_TEAM = 1;

    public static HardwareMap hardwareMap;
    public static DcMotorEx leftDrive = null;
    public static DcMotorEx rightDrive = null;
    public static DcMotorEx elevator = null;
    public static LynxI2cColorRangeSensor colorSensor = null;
    public static Servo left_grab_servo = null;
    public static Servo right_grab_servo = null;
    public static Servo jewel_arm = null;
    public static BNO055IMU imu;
    public static DigitalChannel elevator_switch;
    public static VuforiaLocalizer vuforia;
    public static VuforiaTrackables relicTrackables;
    public static VuforiaTrackable relicTemplate;

    public static double leftTick;
    public static double rightTick;
    public static boolean firstGo;
    public static AveragingFilter leftFilter;
    public static AveragingFilter rightFilter;


    public static GriffinAccelerationIntegratorLowPass integrator;

    public static void init(boolean autonomous, HardwareMap hm) {
        hardwareMap = hm;

        //filters for drive
        leftFilter = new AveragingFilter(0.1);
        rightFilter = new AveragingFilter(0.1);

        //imu params yep
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = true;
        imuParams.calibrationDataFile = "1292017IMUCalibration.json";
        imuParams.loggingTag = "IMU";

        //hardware variables
        leftDrive  = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "right_drive");
        elevator = (DcMotorEx)hardwareMap.get(DcMotor.class, "elevator");
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorSensor = (LynxI2cColorRangeSensor) hardwareMap.get(ColorSensor.class, "color_sensor");
        left_grab_servo = hardwareMap.get(Servo.class, "left_grab_servo");
        right_grab_servo = hardwareMap.get(Servo.class, "right_grab_servo");
        jewel_arm = hardwareMap.get(Servo.class, "jewel_arm");
        elevator_switch = hardwareMap.get(DigitalChannel.class, "elevator_switch");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        integrator = new GriffinAccelerationIntegratorLowPass();
        imuParams.accelerationIntegrationAlgorithm = integrator;
        imu.initialize(imuParams);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //Pid coefficients
        //PIDCoefficients pid_drive = new PIDCoefficients(1000, 0, 0);
        //PIDCoefficients pid_drive = new PIDCoefficients(100, 0, 0);
        PIDCoefficients pid_drive = new PIDCoefficients(50, 0.01, 0);
        Robot.leftDrive.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid_drive);
        Robot.rightDrive.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid_drive);
        PIDCoefficients pid_elevator = new PIDCoefficients(5,5,0);
        Robot.elevator.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid_elevator);

        openAcq();
        Robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (autonomous) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AYnvBxr/////AAAAGct+LQHAOErzsermw13rRThjXOifVm8ucgJ32zD6Poj1Pau0qeQy/hYxMZo0kBiXevd94fmIeHUCY5WH1KvODTTT6LRv8vhF37dT4vbmvzXOvxWPzXbYvjlrzREzZ+Qzagm/0rM1VDeBEVEBiPc540UaQ4xWwpDHNqH51qY5gD2zvZ2divqMh5uVn90TTi7RrwKDLCOv1Ak9C59qmYK0+cFSG42NlZPkc7wmwdId9dtZjRIXC7R4c0EazIVGnWLCWHat7S3FdS0udCobx287lzaE/1nECokxnPxR+ZdXeKUX1Pr+SCeQ7U/Q9LR/dP2EvGOxBfCioE/JZ/tDPOZC8b79Bu3QEHKldWxRmL14k/5k";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            firstGo = true;
        } else {
            firstGo = false;
        }
    }

    /*private void dispatchTakePictureIntent() {
        Intent takePictureIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
        if (takePictureIntent.resolveActivity(hardwareMap.appContext.getPackageManager()) != null) {
            hardwareMap.appContext.startActivity(takePictureIntent);
        }
    }*/

    public static void start() {
        // YES!
        //imu.startAccelerationIntegration(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new Velocity(DistanceUnit.INCH, 0, 0, 0, 0 ), 1000);
    }
    
    
    public static Point getAutoInstructions(String in) {
        // will automatically go center column if it did not read the picto...
        if (in.equals("L")) {
            return Path.LEFT_TURN[0];
        } else if (in.equals("R")) {
            return Path.RIGHT_TURN[0];
        } else {
            return new Point(0, 0);
        }
    }
    
    public static void stop() {
        //music.stop();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public static void driveToNextPoint(Point currentPoint, Point nextPoint, double currentAngle, double power)
    {
        double leftRadius;
        double rightRadius;
        System.out.println("109 currentPoint: " + currentPoint);
        System.out.println("110 nextPoint: " + nextPoint);
        double distanceBetweenPoints = Point.distance(currentPoint, nextPoint);
        System.out.println("111 distanceBetweenPoints: " + distanceBetweenPoints ); //PINEAPPLE, ORANGE,
        double turnAngle = Point.getAngle(currentPoint, nextPoint) - currentAngle;
        if (turnAngle < -90 )
        {
            turnAngle += 180;
        }
        else if (turnAngle > 90 )
        {
            turnAngle -= 180;
        }

        System.out.println("112 turnAngle: " + turnAngle ); //PINEAPPLE, ORANGE,
        Point center = Point.getCenter(distanceBetweenPoints, turnAngle, currentPoint);
        System.out.println("113 center: " + center.x()+ " - " + center.y()); //PINEAPPLE, ORANGE,
        double radius = Point.distance(center, currentPoint);
        System.out.println("114 radius: " + radius); //PINEAPPLE, ORANGE,

        if(turnAngle > 0) {
            leftRadius = radius + (ROBOT_WIDTH / 2);
            System.out.println("115 leftRadius: " + leftRadius); //PINEAPPLE, ORANGE,
            rightRadius = radius - (ROBOT_WIDTH / 2);
            System.out.println("116 rightRadius: " + rightRadius); //PINEAPPLE, ORANGE,
        }
        else if (turnAngle < 0)
        {
            leftRadius = radius - (ROBOT_WIDTH / 2);
            System.out.println("115 leftRadius: " + leftRadius); //PINEAPPLE, ORANGE,
            rightRadius = radius + (ROBOT_WIDTH / 2);
            System.out.println("116 rightRadius: " + rightRadius); //PINEAPPLE, ORANGE,
        }
        else
        {
            leftRadius = radius;
            System.out.println("115 leftRadius: " + leftRadius); //PINEAPPLE, ORANGE,
            rightRadius = radius;
            System.out.println("116 rightRadius: " + rightRadius); //PINEAPPLE, ORANGE,
        }
        double leftDistanceOfArc;
        double rightDistanceOfArc;
        if (turnAngle != 0) {
            leftDistanceOfArc = Point.getArcDistance(leftRadius, turnAngle);
            System.out.println("117 leftDistanceOfArc: " + leftDistanceOfArc ); //PINEAPPLE, ORANGE,
            rightDistanceOfArc = Point.getArcDistance(rightRadius, turnAngle);
            System.out.println("118 rightDistanceOfArc: " +  rightDistanceOfArc); //PINEAPPLE, ORANGE,
        } else {
            leftDistanceOfArc = distanceBetweenPoints;
            rightDistanceOfArc = distanceBetweenPoints;
        }
        driveForwardDistance(power, leftDistanceOfArc, rightDistanceOfArc);
        System.out.println("119 BANANA: " + leftDistanceOfArc + " - " + rightDistanceOfArc); //PINEAPPLE, ORANGE,
    }

    public static void driveForwardTicks(double init_power, double leftDistance, double rightDistance)
    {

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition((int)(leftDistance));
        rightDrive.setTargetPosition((int)(rightDistance));
        leftDrive.setPower(init_power);
        rightDrive.setPower(init_power);
    }

    public static void driveForwardDistance(double power, double leftDistance, double rightDistance) {
        double distanceForOneRotation = (1.5) * (4 * Math.PI);
        double distanceForOneEncoderTick = distanceForOneRotation/DRIVE_ENCODER_TICKS;

        resetDriveEncoders();
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setTargetPosition((int)(leftDistance / distanceForOneEncoderTick));
        rightDrive.setTargetPosition((int)(rightDistance / distanceForOneEncoderTick));

        if (leftDistance > rightDistance)
        {
            leftDrive.setPower(power);
            rightDrive.setPower(power * (rightDistance / leftDistance));
        }
        else if(rightDistance > leftDistance)
        {
            leftDrive.setPower(power * (leftDistance / rightDistance));
            rightDrive.setPower(power);
        }
        else
        {
            driveStraight(power);
        }

        while (Robot.leftDrive.isBusy() && rightDrive.isBusy());

        driveStraight(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public static void driveForwardDistance(double power, double distance) {

        double distanceForOneRotation = (1.5) * (4 * Math.PI);
        double distanceForOneEncoderTick = distanceForOneRotation/DRIVE_ENCODER_TICKS;

        resetDriveEncoders();

        leftDrive.setTargetPosition((int)(distance / distanceForOneEncoderTick));
        rightDrive.setTargetPosition((int)(distance / distanceForOneEncoderTick));

        driveStraight(power);

        while (Robot.leftDrive.isBusy() && rightDrive.isBusy());

        driveStraight(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void moveElevatorToTick(double power, int tick) {
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(tick);
        elevator.setPower(power);
    }

    public static void elevatorPreset(int pos) {
        switch (pos) {
            case 0:
                moveElevatorToTick(1,0);
                break;
            case 1:
                moveElevatorToTick(1, 3900); //3391
                break;
            case 2:
                moveElevatorToTick(1, 6295); //6095
                break;
            case 3: //driving mode
                moveElevatorToTick(1, 540);
            break;
        }
    }

    public static boolean isCloseEnough(Point p) {
        double leftDif = Math.abs(leftDrive.getCurrentPosition() - p.x());
        double rightDif = Math.abs(rightDrive.getCurrentPosition() - p.y());
        //System.out.println("BARRY: " + leftDif + " <> " + rightDif);
        if (leftDif < CLOSE_THRESHOLD && rightDif < CLOSE_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    public static void lowerArm(double adjust) {
        jewel_arm.setPosition(0.9 + adjust);
    }

    public static  void raiseArm() {
        jewel_arm.setPosition(0.25);
    }

    public static boolean isBallTeamColor(double[] color, int n) {
        switch (n) {
            case RED_TEAM:
                return Utils.isGemRed(color[0], color[1], color[2]) && !Utils.isGemBlue(color[0], color[1], color[2]);
            case BLUE_TEAM:
                return !Utils.isGemRed(color[0], color[1], color[2]) && Utils.isGemBlue(color[0], color[1], color[2]);
            default:
                return false;
        }
    }
    public static void driveStraight(double power) {
        //resetDriveEncoders();
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public static void closeAcq() {
        left_grab_servo.setPosition(0.78);
        right_grab_servo.setPosition(0.26);
    }
    
    public static void halfAcq() {
        left_grab_servo.setPosition(0.65);
        right_grab_servo.setPosition(0.35);
    }

    public static void openAcq() {
        left_grab_servo.setPosition(0.55);
        right_grab_servo.setPosition(0.45);
    }

    public static void stopDrives() {
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        resetDriveEncoders();
    }

    public static void resetDriveEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static boolean seesBall() {
        return colorSensor.getDistance(DistanceUnit.INCH) < 3.0;
    }
}
