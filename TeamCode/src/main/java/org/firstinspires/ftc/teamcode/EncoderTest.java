package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Encoder test")
public class EncoderTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    public void runOpMode() {

        robot.init(hardwareMap);
        waitForStart();

        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDriveFront.setTargetPosition(10);
        robot.rightDriveFront.setTargetPosition(10);

        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDriveFront.setPower(1);
        robot.rightDriveFront.setPower(1);

        while (robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()) {

        }

        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);

//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
