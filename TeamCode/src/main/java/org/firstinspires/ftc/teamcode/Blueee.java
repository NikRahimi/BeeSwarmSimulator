package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "BLEW", group = "Competition")
public class Blueee extends OpMode {

    // -------------------- VARIABLES --------------------

    MecanumDrive drive;
    IMU imu; // Direct IMU instance

    // Hardware init
    private DcMotorEx intakeMotor;
    private DcMotorEx flyMotor;
    private DcMotorEx flyMotor2;
    private Servo keeperServo;
    private Servo flowerServo;
    private Servo flowerServo2;

    // AprilTag ID number (20 -> blue, 24 -> red)
    int tagNum = 20;

    // PIDF tuning variables
    double kP = 0.058;
    double angleTolerance = 2;
    double kD = 0.0;
    double kF = 0.4;

    // Miscellaneous variables
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double currentTime = 0;
    double lastTime = 0;

    // Flywheel variables
    double targetRPM = 3700.0;
    final double TICKS_PER_REV_FLY = 28.0;
    double maxRPM = 6000.0;
    double ticksPerSecond = 0.0;
    double maxTPS = 0.0;

    // Servo toggle variables for flower servos
    boolean flowerState = false;
    boolean xLastState = false;

    // Driving setup
    double forward, strafe, rotate;
    double maxMotorPower = 0.7;

    // Controller-customized PIDF tuning steps
    double[] stepSizes = {0.1, 0.001, 0.0001};
    int stepIndex = 1;

    // ---------------------------------------------------

    // --------------------- MAIN LOOP ----------------------
    @Override
    public void loop() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Keep Road Runner updating its internal tracking
        drive.updatePoseEstimate();

        // Retrieve the current pose safely from the localizer
        Pose2d currentPose = drive.localizer.getPose();

        // ---------------------- ROBOT-CENTRIC DRIVE ----------------------
        forward = -gamepad1.left_stick_y;
        rotate  = gamepad1.left_stick_x;
        strafe  = gamepad1.right_stick_x;

        drive.drive(forward, strafe, rotate);
        // -----------------------------------------------------------------

        // ---------------------- Mechanism Inputs -------------------------

        // Flywheel Target RPM control via right bumper
        if (gamepad1.right_bumper) {
            targetRPM = 3150;
        } else {
            targetRPM = 0.0;
        }

        // Flower Servo Toggle Logic using Gamepad 1 X Button
        boolean xPressedNow = gamepad1.x;
        if (xPressedNow && !xLastState) {
            flowerState = !flowerState; // Flip toggle state
        }
        xLastState = xPressedNow;

        // Apply positions based on the toggle state (using valid 0.0 - 1.0 ranges)
        if (flowerState) {
            flowerServo.setPosition(0.75);   // Active position 1
            flowerServo2.setPosition(0.75);  // Active position 2
        } else {
            flowerServo.setPosition(0.1);   // Home / Default position 1
            flowerServo2.setPosition(0.1);  // Home / Default position 2
        }

        maxRPM = 6000.0;
        ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
        maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;

        // Drive flywheels via velocity PID
        flyMotor.setVelocity(ticksPerSecond);
        flyMotor2.setVelocity(ticksPerSecond);

        // Intake control on Gamepad 1 A Button
        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        // Keeper servo control on Gamepad 1 B Button
        if (gamepad1.b) {
            keeperServo.setPosition(0.75);
        } else {
            keeperServo.setPosition(0.55);
        }

        // FTC Dashboard Telemetry Streaming
        double actualRPM1 = (flyMotor.getVelocity() * 60.0) / TICKS_PER_REV_FLY;
        double actualRPM2 = (flyMotor2.getVelocity() * 60.0) / TICKS_PER_REV_FLY;
        packet.put("Target RPM", targetRPM);
        packet.put("Actual RPM (Motor 1)", actualRPM1);
        packet.put("Actual RPM (Motor 2)", actualRPM2);
        dashboard.sendTelemetryPacket(packet);

        // -----------------------------------------------------------------

        // PD tuning inputs (Gamepad 2)
        if (gamepad2.bWasPressed()) {stepIndex = (stepIndex + 1) % stepSizes.length;}
        if (gamepad2.dpadLeftWasPressed()) {kP -= stepSizes[stepIndex];}
        if (gamepad2.dpadRightWasPressed()) {kP += stepSizes[stepIndex];}
        if (gamepad2.dpadUpWasPressed()) {kD += stepSizes[stepIndex];}
        if (gamepad2.dpadDownWasPressed()) {kD -= stepSizes[stepIndex];}
        if (gamepad2.y) {kF += stepSizes[stepIndex];}
        if (gamepad2.a) {kF -= stepSizes[stepIndex];}

        // Driver Station Telemetry
        double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Heading (Deg)", "%.2f", headingDeg);
        telemetry.addLine("----------------------");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("TPS", ticksPerSecond);
        telemetry.addData("Leader Power", "%.2f", flyMotor.getPower());
        telemetry.addData("Flower State", flowerState ? "Active" : "Home");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", kD);
        telemetry.addData("Tuning F", "%.4f", kF);
        telemetry.addData("Step Size", "%.4f (B)", stepSizes[stepIndex]);
        telemetry.addData("RR X (Inches)", "%.2f", currentPose.position.x);
        telemetry.addData("RR Y (Inches)", "%.2f", currentPose.position.y);
        telemetry.addData("RR Heading (Deg)", "%.2f", Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.update();
    }
    // --------------------------------------------------

    // ------------------ INIT METHODS ----------------------
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.init();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        keeperServo = hardwareMap.get(Servo.class, "keeperServo");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyMotor");
        flyMotor2 = hardwareMap.get(DcMotorEx.class, "flyMotor2");
        flowerServo = hardwareMap.get(Servo.class, "flowerServo");
        flowerServo2 = hardwareMap.get(Servo.class, "flowerServo2");

        // Leader motor manages velocity PID with encoder active
        flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyMotor.setVelocityPIDFCoefficients(60, 2, 0, 11.7);

        // Follower motor runs velocity PID to match leader
        flyMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyMotor2.setVelocityPIDFCoefficients(60, 2, 0, 11.7);

        flyMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flowerServo2.setDirection(Servo.Direction.REVERSE);

        flowerServo.setPosition(0.1);   // Active position 1
        flowerServo2.setPosition(0.1);  // Active position 2

        telemetry.addLine("Initialized Successfully");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        currentTime = getRuntime();
        xLastState = false;
    }
}