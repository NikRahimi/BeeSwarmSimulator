package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Telemetry Test Op", group = "Tests")
public class TelemetryTestOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Hello World", 12345);
            telemetry.update();
            sleep(2000); // Pause for 2 seconds to view the telemetry
        }
    }
}
