package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by andrew on 10/27/17.
 */

public class AutonomousEncoder extends LinearOpMode {

    DcMotor motorRight = null;
    DcMotor motorLeft = null;
    HardwareRobot robot = new HardwareRobot();
    HardwareMap hwMap = null;
    //hello!

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hwMap);

    }

    public void drive(int leftPower, int rightPower, int inches) throws InterruptedException {

        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);
        robot.leftDriveFront.setTargetPosition((int) (robot.COUNTS_PER_INCH) * inches);
        robot.rightDriveFront.setTargetPosition((int) (robot.COUNTS_PER_INCH) * inches);
        robot.encoderSwitch();
        robot.leftDriveFront.setPower(leftPower);
        robot.rightDriveFront.setPower(rightPower);

    }

}
