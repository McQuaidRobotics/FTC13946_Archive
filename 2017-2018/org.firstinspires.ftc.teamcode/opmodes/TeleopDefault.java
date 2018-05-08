/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.tools.AveragingFilter;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopDefault", group="Iterative Opmodes")
public class TeleopDefault extends OpMode
{
     // in inches

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private double elevator_power;
    private boolean elevatorIsResetting;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        Robot.init(false, hardwareMap);
        Robot.elevator.setPower(-0.5);
        elevatorIsResetting = true;
    }

    @Override
    public void init_loop() {
        if (elevatorIsResetting && !Robot.elevator_switch.getState()) {
            elevatorIsResetting = false;
            Robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.elevator.setPower(0);
        }
    }

    @Override
    public void start() {
        runtime.reset();
        Robot.start();
        //Robot.music.stop();
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // IO
        // Tank Mode uses one stick to control each wheel.
        leftPower  = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;
        if (gamepad1.left_trigger > 0) leftPower = 0.1 * gamepad1.left_trigger + 0.2;
        if (gamepad1.right_trigger > 0) rightPower = 0.1 * gamepad1.right_trigger + 0.2;

        if (elevatorIsResetting && !Robot.elevator_switch.getState()) {
            elevatorIsResetting = false;
            Robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.elevator.setPower(0);
        }

        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            Robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.dpad_up) {
                Robot.elevator.setPower(1);
            }
            else if (gamepad1.dpad_down) {
                Robot.elevator.setPower(-1);
            }
        }

        if (gamepad1.left_bumper) Robot.openAcq();
        if (gamepad1.right_bumper) Robot.closeAcq();

        if (gamepad1.a) {
            elevatorIsResetting = true;
            Robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.elevator.setPower(-1);
        }

        if (gamepad1.x) {
            elevatorIsResetting = false;
            Robot.elevatorPreset(1);
        }

        if (gamepad1.y) {
            elevatorIsResetting = false;
            Robot.elevatorPreset(2);
        }

        if (gamepad1.b) {
            //MediaPlayer airhorn = MediaPlayer.create(hardwareMap.appContext, R.raw.airhorn);
            //airhorn.start();
            Robot.elevatorPreset(3);
            /*System.out.println("pos1 " + Robot.leftDrive.getCurrentPosition() + "-"+ Robot.rightDrive.getCurrentPosition());
            Robot.leftTick = Robot.leftDrive.getCurrentPosition();
            Robot.rightTick = Robot.rightDrive.getCurrentPosition();
            Robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        }



        Robot.leftDrive.setPower(Robot.leftFilter.filter(leftPower));
        Robot.rightDrive.setPower(Robot.rightFilter.filter(rightPower));

        //if (gamepad1.dpad_up) Robot.driveForwardDistance(0.6, 4);
        //if (gamepad1.dpad_down) Robot.driveForwardDistance(0.6, -4);
        telemetry.addData("ticks:", Robot.rightTick + " - " + Robot.leftTick);
        //Quaternion quat = Robot.imu.getQuaternionOrientation();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        //telemetry.addData("Encoders", "left (%.2f in), right (%.2f in), elevator (%.2f in)", Utils.encoderToDistance(Robot.leftDrive.getCurrentPosition()), Utils.encoderToDistance(Robot.rightDrive.getCurrentPosition()), Utils.encoderToDistance(Robot.elevator.getCurrentPosition()));
        telemetry.addData("MotorEncoders:", "leftDrive = " + Robot.leftDrive.getCurrentPosition() + " ; rightDrive = " + Robot.rightDrive.getCurrentPosition() + " ;");
        //telemetry.addData("Color", "red (%d), green (%d), blue (%d), alpha (%d), hue (%d)", Robot.colorSensor.red(), Robot.colorSensor.green(), Robot.colorSensor.blue(), Robot.colorSensor.alpha(), (int)Utils.rgbToHue(Robot.colorSensor.red(), Robot.colorSensor.green(), Robot.colorSensor.blue()));
        //telemetry.addData("Gyro", "%.2f (x); %.2f (y); %.2f (z); %.2f (w)", quat.x, quat.y, quat.z, quat.w);
        telemetry.addData("Angle", Robot.imu.getAngularOrientation());
        //telemetry.addData("Acceleration", Robot.imu.getLinearAcceleration());
        //telemetry.addData("Temperature", "%.2f (%s)", imu.getTemperature().temperature, imu.getTemperature().unit.toString());
        //telemetry.addData("Velocity", imu.getVelocity());
        //telemetry.addData("Position", Robot.imu.getPosition().toUnit(DistanceUnit.INCH));
        telemetry.addData("Elevator Tick", Robot.elevator.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Robot.stop();
    }
}
