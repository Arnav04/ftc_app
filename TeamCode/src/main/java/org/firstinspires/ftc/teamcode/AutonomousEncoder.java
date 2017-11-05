package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonomousEncoder extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    HardwareMap hwMap = null;
    //hello!

    public void runOpMode() throws InterruptedException {

        robot.init(hwMap);

    }

    public void drive(int leftPower, int rightPower, int inches) throws InterruptedException {

        robot.setAllLeftDrivePower(0);
        robot.setAllRightDrivePower(0);

        robot.leftDriveFront.setTargetPosition((int) (robot.COUNTS_PER_INCH) * inches);
        robot.rightDriveFront.setTargetPosition((int) (robot.COUNTS_PER_INCH) * inches);
        robot.leftDriveBack.setTargetPosition((int) (robot.COUNTS_PER_INCH) * inches);
        robot.rightDriveBack.setTargetPosition((int) (robot.COUNTS_PER_INCH) * inches);

        robot.encoderSwitch();

        robot.setAllLeftDrivePower(leftPower);
        robot.setAllRightDrivePower(rightPower);

    }
}
