package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous (name = "Vinay's mom")
public class ServoTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);
        waitForStart();

        /*robot.cvn1.setPower(1);
        robot.cvn2.setPower(1);
        Thread.sleep(1000);

        robot.cvn1.setPower(0);
        robot.cvn2.setPower(0);
        Thread.sleep(10000);
        stop();

        //while (opModeIsActive())
        //{

        //robot.leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //robot.encoderSwitch();

        /*int current = robot.rightDriveFront.getCurrentPosition();
        int target = current + (int)(robot.COUNTS_PER_INCH * 50);
        telemetry.addData("run mode: ",robot.rightDriveFront.getMode());
        telemetry.addData("current pos", robot.rightDriveFront.getCurrentPosition());
        telemetry.update();

        robot.rightDriveFront.setTargetPosition(target);
        robot.rightDriveBack.setTargetPosition(target);
        robot.leftDriveFront.setTargetPosition(target);
        robot.leftDriveBack.setTargetPosition(target);
        telemetry.addData("target pos: ",robot.rightDriveFront.getTargetPosition());

        robot.setAllLeftDrivePower(1);
        robot.setAllRightDrivePower(1);
        while( Math.abs(target - robot.leftDriveFront.getCurrentPosition()) > 100 && Math.abs(target - robot.rightDriveFront.getCurrentPosition()) > 100 && Math.abs(target - robot.leftDriveBack.getCurrentPosition()) > 100 && Math.abs(target - robot.rightDriveBack.getCurrentPosition()) > 100)
        {

        }
        //robot.rightDriveFront.
        //robot.rightDriveFront.setPower(0);
        robot.setAllLeftDrivePower(0.5);
        robot.setAllRightDrivePower(0.5);

        while( Math.abs(target - robot.leftDriveFront.getCurrentPosition()) > 50 && Math.abs(target - robot.rightDriveFront.getCurrentPosition()) > 50 && Math.abs(target - robot.leftDriveBack.getCurrentPosition()) > 50 && Math.abs(target - robot.rightDriveBack.getCurrentPosition()) > 50)
        {

        }
        //robot.rightDriveFront.
        //robot.rightDriveFront.setPower(0);
        robot.setAllLeftDrivePower(0.25);
        robot.setAllRightDrivePower(0.25);

        while( Math.abs(target - robot.leftDriveFront.getCurrentPosition()) > 1 && Math.abs(target - robot.rightDriveFront.getCurrentPosition()) > 1 && Math.abs(target - robot.leftDriveBack.getCurrentPosition()) > 1 && Math.abs(target - robot.rightDriveBack.getCurrentPosition()) > 1)
        {

        }
        //robot.rightDriveFront.
        //robot.rightDriveFront.setPower(0);
        robot.setAllLeftDrivePower(0);
        robot.setAllRightDrivePower(0);

        telemetry.addData("current pos", robot.rightDriveFront.getCurrentPosition());
        telemetry.update();
        Thread.sleep(10000);
        stop();*/

    }
}
