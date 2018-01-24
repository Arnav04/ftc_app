package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by nihalmahajani on 1/8/18.
 */

@TeleOp(name="Debug Teleop")
public class TeleOperation extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double threshold = 0.05;
            double leftPower;
            double rightPower;

            double gPadLeftY = gamepad1.left_stick_y;
            double gPadRightY = gamepad1.right_stick_y;

            leftPower = Range.clip(gPadLeftY, -1.0, 1.0);
            rightPower = Range.clip(gPadRightY, -1.0, 1.0);

            robot.setAllLeftDrivePower(1);
            robot.setAllRightDrivePower(1);
            if (Math.abs(leftPower) > threshold) {
                robot.setAllLeftDrivePower(leftPower);
            }
            else if (Math.abs(leftPower) <= threshold) {
                if (robot.leftDriveBack.getPower() > threshold) {
                    robot.setAllLeftDrivePower(robot.leftDriveBack.getPower() - 0.1);
                }
                else {
                    robot.setAllLeftDrivePower(0);
                }
            }
            if (Math.abs(rightPower) > threshold) {
                robot.setAllRightDrivePower(rightPower);
            }
            else if (Math.abs(rightPower) <= threshold) {
                if (robot.rightDriveBack.getPower() > threshold) {
                    robot.setAllRightDrivePower(robot.rightDriveBack.getPower() - 0.1);
                }
                else {
                    robot.setAllRightDrivePower(0);
                }
            }
        }
    }
}

