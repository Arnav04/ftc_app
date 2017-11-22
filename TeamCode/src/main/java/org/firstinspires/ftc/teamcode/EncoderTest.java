package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous (name = "Encoder test")
public class EncoderTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);
        waitForStart();

        robot.driveWithEncoder(10,1,1);

    }
}
