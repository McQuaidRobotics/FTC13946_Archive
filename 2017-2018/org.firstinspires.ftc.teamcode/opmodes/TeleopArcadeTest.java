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
import org.firstinspires.ftc.teamcode.tools.Utils;

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

@TeleOp(name="TeleopArcadeTest", group="Linear Opmodes")
public class TeleopArcadeTest extends OpMode
{
    // in inches

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private double elevator_power;
    private boolean elevatorIsResetting;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        Robot.init(false, hardwareMap);
        Robot.elevator.setPower(-0.5);
        elevatorIsResetting = true;
        gamepad1.setJoystickDeadzone(0.1f);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (elevatorIsResetting && !Robot.elevator_switch.getState()) {
            elevatorIsResetting = false;
            Robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.elevator.setPower(0);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        Robot.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {// IO
        // Tank Mode uses one stick to control each wheel.
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

        if (gamepad1.left_bumper && gamepad1.right_bumper) Robot.halfAcq();
        else if (gamepad1.left_bumper) Robot.openAcq();
        else if (gamepad1.right_bumper) Robot.closeAcq();

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
            Robot.elevatorPreset(3);
        }
        
        if (gamepad1.dpad_right) {
            System.out.println("BEANIE: ( " + Robot.leftDrive.getCurrentPosition() + " , " + Robot.rightDrive.getCurrentPosition() + " )");
            Robot.resetDriveEncoders();
        }

        double throttle = -gamepad1.left_stick_y;
        double turnValue = gamepad1.right_stick_x;
        double leftPower = throttle + turnValue;
        double rightPower = throttle - turnValue;

        if (gamepad1.left_trigger > 0) leftPower = 0.2 * gamepad1.left_trigger + 0.3;
        if (gamepad1.right_trigger > 0) rightPower = 0.2 * gamepad1.right_trigger + 0.3;

        Robot.leftDrive.setPower(Robot.leftFilter.filter(leftPower));
        Robot.rightDrive.setPower(Robot.rightFilter.filter(rightPower));
        //telemetry.addData("left, right", Robot.leftFilter.getVal() + " ; " + Robot.rightFilter.getVal());
        telemetry.addData("motor encoders", Robot.leftDrive.getCurrentPosition() + " ; " + Robot.rightDrive.getCurrentPosition());
    }

    @Override
    public void stop() {
        Robot.stop();
    }
}
