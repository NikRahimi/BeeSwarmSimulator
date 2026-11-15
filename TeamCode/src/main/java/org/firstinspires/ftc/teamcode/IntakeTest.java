package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BeeSwarm ;)", group = "Drive")
public class IntakeTest extends LinearOpMode {

    private CRServo intakeServoOne;

    private CRServo intakeServoTwo;

    private DcMotorEx leMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your MecanumDrive class.
        // The Pose2d represents the robot's starting position.
        // For tele-op, this is usually (0, 0) with a 0-degree heading.
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        leMotor =hardwareMap.get(DcMotorEx.class, "leMotor"); // initialze le motor
        intakeServoOne = hardwareMap.get(CRServo.class, "intake1"); // Initialize the intake
        intakeServoTwo = hardwareMap.get(CRServo.class, "intake2"); // Intialize the intake

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // Read gamepad inputs for drive motion.
            // The negative signs are because the FTC SDK's gamepad axes are inverted.
            // Left stick controls linear movement (forward/backward and strafing).
            // Right stick controls rotational movement.
            double y = -gamepad1.left_stick_y; // "Forward" and "Backward"
            double x = -gamepad1.left_stick_x; // "Left" and "Right" strafing
            double rx = -gamepad1.right_stick_x; // "Rotation"


            double speedmultiplier = 2.0;

            double mechanismPower = gamepad2.right_stick_y * speedmultiplier;
            if (leMotor != null) {
                leMotor.setPower(mechanismPower);
            }



            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                // Spin both intake servos inward
                intakeServoOne.setPower(-1.0);
                intakeServoTwo.setPower(-1.0);// Spin one direction
            } else {
                // Stop the servos when stick is released
                intakeServoOne.setPower(0.0);
                intakeServoTwo.setPower(0.0);
            }

            // --- Proportional Control Logic ---

// Define a "deadzone" to prevent tiny movements.
            double joystickDeadzone = 0.1;

// Read the joystick value once.
            double stickValue = gamepad2.left_stick_x;

            if (Math.abs(stickValue) > joystickDeadzone) {
                intakeServoTwo.setPower(stickValue);
            } else {
                // If the stick is inside the deadzone, stop the servo.
                intakeServoTwo.setPower(0.0);
            }
        }
    }
}
