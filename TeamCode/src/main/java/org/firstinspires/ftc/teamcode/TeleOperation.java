package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOperation")
public class TeleOperation extends LinearOpMode {

     HardwareRobot robot = new HardwareRobot();
     HardwareMap hwmap = null;

    public void runOpMode() throws InterruptedException {

        robot.init(hwmap);

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            if (gamepad1.right_trigger > 0)
            {

                robot.cvn1.setPower(1);
                robot.cvn2.setPower(-1);

            } else {

                robot.cvn1.setPower(0);
                robot.cvn2.setPower(0);

            }

            if (gamepad1.dpad_up)
            {
                robot.elev1.setPower(1);
                robot.elev2.setPower(2);
            } else if (gamepad1.dpad_down)
            {
                robot.elev1.setPower(-1);
                robot.elev2.setPower(-1);
            } else {
                robot.elev1.setPower(0);
                robot.elev2.setPower(0);
            }

            double gPadLeftY = gamepad2.left_stick_y;
            double gPadRightY = gamepad2.right_stick_y;

            leftPower = Range.clip(gPadLeftY, -1.0, 1.0);
            rightPower = Range.clip(gPadRightY, -1.0, 1.0);

            if (gPadLeftY == 0 && gPadRightY == 0) {

                while ( leftPower >= 0.5 && rightPower >= 0.5 ) {

                    leftPower -=  0.1;
                    rightPower -= 0.1;

                    robot.setAllLeftDrivePower(leftPower);
                    robot.setAllRightDrivePower(rightPower);

                }

                leftPower = 0;
                rightPower = 0;

            }

            robot.setAllLeftDrivePower(leftPower);
            robot.setAllRightDrivePower(rightPower);

        }
    }
}
