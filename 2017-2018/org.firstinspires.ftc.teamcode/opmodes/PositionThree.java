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

@Autonomous(name="Position 3 - Blue Alliance", group="Iterative Opmode")
public class PositionThree extends PositionOne {
    @Override
    public void pathInit() {
        team = Robot.BLUE_TEAM;
        path = new Path(Path.POSITION_THREE_PATH);
    }
}