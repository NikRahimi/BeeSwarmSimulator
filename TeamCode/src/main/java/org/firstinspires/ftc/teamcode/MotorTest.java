package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Test - Right Back", group = "T    est")
public class MotorTest extends LinearOpMode {

    private DcMotorEx rightBack;

    @Override
    public void runOpMode() {
        // Get the motor from the hardware map
        // Make SURE the name "rightBack" is EXACTLY what is in your configuration
        try {
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find motor 'rightBack'. Check configuration.");
            telemetry.update();
            sleep(5000);
            return; // Stop the OpMode if the motor isn't found
        }

        // Set the motor to run without encoders for this simple test
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized. Press Play to start test.");
        telemetry.addData("Info", "Press 'A' to run motor forward, 'B' for reverse.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // Run the motor forward at 50% power
                rightBack.setPower(0.5);
                telemetry.addData("Motor Power", "0.5 (Forward)");
            } else if (gamepad1.b) {
                // Run the motor backward at 50% power
                rightBack.setPower(-0.5);
                telemetry.addData("Motor Power", "-0.5 (Reverse)");
            } else {
                // Stop the motor if no button is pressed
                rightBack.setPower(0);
                telemetry.addData("Motor Power", "0.0 (Stopped)");
            }
            telemetry.update();
        }
    }
}
    