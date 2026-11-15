package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp (name = "Motor Test", group = "Test")
public class MotorTestOp extends LinearOpMode {

    private DcMotorEx testMotor;

    @Override
    public void runOpMode()  {

        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
            // Wait for the game to start (driver presses PLAY)

        waitForStart();

        while (opModeIsActive()) {

            testMotor.setPower(0.1); // Set the motor to run at half power

            telemetry.addData("Status", "Running Motor Test");
            telemetry.update();
        }
    }

}
