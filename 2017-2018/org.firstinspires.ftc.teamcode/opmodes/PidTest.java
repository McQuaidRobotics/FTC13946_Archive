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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.tools.Path;
import org.firstinspires.ftc.teamcode.tools.Point;
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

@Autonomous(name="PidTest", group="Iterative Opmode")
public class PidTest extends OpMode
{

    private boolean elevatorIsResetting;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gametime = new ElapsedTime();
    private int state;
    private Path path = new Path();
    private String picto_column = "nothing";

    // states
    private final static int GRAB_ELEVATE = 0;
    private final static int PICTO_CHECK = 1;
    private final static int COLOR_CHECK = 2;
    private final static int FOLLOW_PATH = 3;
    private final static int BACKAWAY = 4;
    private final static int REALIGN = 5;
    private final static int FORWARD = 6;
    private final static int STOP = 7;

    // team constants
    private int team;

    private double adjustment = -0.4;
    private double delta = 0.01;

    //test values
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        team = Robot.RED_TEAM;
        path = new Path(Path.STRAIGHT_PATH);

        telemetry.addData("Status", "Initialized");

        Robot.init(true, hardwareMap);
        Robot.elevator.setPower(-0.5);
        elevatorIsResetting = true;
        state = 0;
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
        // YES!
        Robot.start();
        //Robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        runtime.reset();
        Robot.relicTrackables.activate();
        gametime.startTime();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("State", state);
        telemetry.addData("vuMark", picto_column);
        double[] color = { Robot.colorSensor.red(), Robot.colorSensor.green(), Robot.colorSensor.blue() };
        switch (state) {
            case GRAB_ELEVATE:
                Robot.closeAcq();
                Robot.elevatorPreset(1);
                //while (Robot.elevator.isBusy());
                changeState(COLOR_CHECK);
                break;
            case COLOR_CHECK:
                if (adjustment == -0.4) Robot.lowerArm(adjustment);
                if (runtime.milliseconds() < 500) break;
                if (Robot.seesBall()) {
                    if (!Robot.isBallTeamColor(color, team)) {
                        double start_time = runtime.milliseconds();
                        while (runtime.milliseconds() - start_time < 250)  {
                            telemetry.addData("Sees", "red");
                            Robot.leftDrive.setPower(-0.45 * (runtime.milliseconds() - start_time) / 250);
                            Robot.rightDrive.setPower(0.45 * (runtime.milliseconds() - start_time) / 250);
                        }
                        Robot.raiseArm();
                        while (runtime.milliseconds() - start_time < 500)  {
                            telemetry.addData("Sees", "red");
                            Robot.leftDrive.setPower(0.45 * (runtime.milliseconds() - start_time) / 500);
                            Robot.rightDrive.setPower(-0.45 * (runtime.milliseconds() - start_time) / 500);
                        }
                        Robot.stop();
                        changeState(PICTO_CHECK);

                    } else if (Robot.isBallTeamColor(color, team)) {
                        double start_time = runtime.milliseconds();
                        while (runtime.milliseconds() - start_time < 250) {
                            telemetry.addData("Sees", "blue");
                            Robot.rightDrive.setPower(-0.45 * (runtime.milliseconds() - start_time) / 250);
                            Robot.leftDrive.setPower(0.45 * (runtime.milliseconds() - start_time) / 250);
                        }
                        Robot.raiseArm();
                        while (runtime.milliseconds() - start_time < 500) {
                            telemetry.addData("Sees", "blue");
                            Robot.rightDrive.setPower(0.45 * (runtime.milliseconds() - start_time) / 500);
                            Robot.leftDrive.setPower(-0.45 * (runtime.milliseconds() - start_time) / 500);
                        }
                        Robot.stop();
                        changeState(PICTO_CHECK);
                    }
                } else {
                    adjustment = Utils.clamp(-0.4, 1, adjustment + delta);
                    Robot.lowerArm(adjustment);
                }
                if (runtime.milliseconds() > 5000) {
                    Robot.raiseArm();
                    changeState(PICTO_CHECK);
                }
                break;
            case PICTO_CHECK:
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(Robot.relicTemplate);
                /*if (vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.milliseconds() < 333) {
                    Robot.leftDrive.setPower(-0.4 * (runtime.milliseconds() / 333));
                    Robot.rightDrive.setPower(0.4 * (runtime.milliseconds() / 333));
                } else if (vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.milliseconds() < 666) {
                    Robot.stop();
                } else if (vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.milliseconds() < 999) {
                    Robot.leftDrive.setPower(0.4 * (runtime.milliseconds() - 666) / 333);
                    Robot.rightDrive.setPower(-0.4 * (runtime.milliseconds() - 666) / 333);
                }*/
                if (vuMark != RelicRecoveryVuMark.UNKNOWN || runtime.milliseconds() >= 1000) {
                    picto_column = vuMark.toString();
                    Robot.stop();
                    changeState(REALIGN);
                }
                break;
            case REALIGN:
                /*if (Robot.imu.getAngularOrientation().firstAngle < -5) {
                    Robot.leftDrive.setPower(-0.25);
                    Robot.rightDrive.setPower(0.25);
                } else if (Robot.imu.getAngularOrientation().firstAngle > 5) {
                    Robot.leftDrive.setPower(0.25);
                    Robot.rightDrive.setPower(-0.25);
                } else {
                    if (runtime.milliseconds() > 1000) {
                        Robot.stop();
                        changeState(FOLLOW_PATH);
                    }
                }

                if (runtime.milliseconds() > 5000) {
                    Robot.stop();
                    changeState(FOLLOW_PATH);
                }*/
                changeState(FOLLOW_PATH);
                break;
            case FOLLOW_PATH:
                double pow = 0.3;
                System.out.println("BOOP1");
                if (path.getIndex() == 0 && Robot.firstGo) {
                    Robot.driveForwardTicks(pow, path.peek().x(), path.peek().y());
                    Robot.firstGo = false;
                    System.out.println("BOOP2");
                }
                if (path.peek().x() == 0 && path.peek().y() == 0) {
                    changeState(FORWARD);
                    System.out.println("BOOP3");
                } else if (path.peek() != null && Robot.isCloseEnough(path.peek())) {
                    Robot.stop();
                    path.inc();
                    if (path.peek() != null) {
                        Point next = path.peek();
                        Robot.driveForwardTicks(pow, next.x(), next.y());
                        System.out.println("COCONUT: going to ( " + next.x() + " , " + next.y() + " )");
                    }
                }

                if (runtime.milliseconds() >= 15000) {
                    Robot.stop();
                    Robot.openAcq();
                    //Robot.driveForwardTicks(pow, -400, -400);
                    //Robot.elevatorPreset(0);
                    changeState(STOP);
                }
                break;
            case FORWARD:
                Robot.stop();
                /*if (gametime.milliseconds() < 27500 && runtime.milliseconds() < 2000) {
                    Robot.driveStraight(0.25);
                } else {
                    Robot.stop();
                    Robot.openAcq();
                    changeState(BACKAWAY);
                }*/
                break;
            case BACKAWAY:
                /*if (runtime.milliseconds() < 500) {
                    Robot.driveStraight(-0.25);
                } else {
                    Robot.stop();
                }

                if (gametime.milliseconds() > 27500) {
                    Robot.stop();
                    Robot.elevatorPreset(0);
                    changeState(STOP);
                }*/
                //comment
                break;
            case STOP:
                break;
        }
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Robot.stop();
    }

    private void changeState(int nextState) {
        if (nextState != state) {
            Robot.resetDriveEncoders();
            state = nextState;
            runtime.reset();
        }
    }

}
