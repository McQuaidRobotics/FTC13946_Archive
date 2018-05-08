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

@Autonomous(name="Position 2 - Red Alliance", group="Iterative Opmode")
public class PositionTwo extends PositionOne {
    @Override
    public void pathInit() {
        team = Robot.RED_TEAM;
        path = new Path(Path.POSITION_TWO_PATH);
    }
}