package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BLEW", group = "Competition")
public class Blueee extends OpMode {

    // -------------------- VARIABLES --------------------

    MecanumDrive drive;
    IMU imu; // Direct IMU instance to avoid pose conflicts

    // Hardware init
    private DcMotorEx intakeMotor;
    private DcMotorEx flyMotor;
    private DcMotorEx flyMotor2;
    private Servo keeperServo;

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

    // Servo positions
    private static final double FLICKER_DEFAULT_POS = 0.0;
    private static final double FLICKER_PRESSED_POS = 0.4;
    double dx, dy, dz, tagDistanceM = 0.0;
    boolean xNow;
    boolean xLast = false;
    boolean isUp = false;

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
        // 1. Keep Road Runner updating its internal dead-wheels tracking
        drive.updatePoseEstimate();

        flyMotor.setVelocityPIDFCoefficients(80, 0, 10, 0); // Flywheel PID

        // ---------------------- FIELD-CENTRIC DRIVE MATH ----------------------
        // Read raw controller inputs (Invert Y because standard joysticks are pushed negative-up)
        double rawForward = -gamepad1.left_stick_y;
        double rawStrafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Reset heading calibration if driver presses the options/start button
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Fetch robot heading directly in radians from the Hub IMU (bypasses pose errors)
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate movement vector components counter to the robot's current heading angle
        forward = rawForward * Math.cos(-botHeading) - rawStrafe * Math.sin(-botHeading);
        strafe  = rawForward * Math.sin(-botHeading) + rawStrafe * Math.cos(-botHeading);
        // ----------------------------------------------------------------------

        // Send field-centric inputs to drive class
        drive.drive(forward, strafe, -rotate);

        // ---------------------- Mechanism Inputs ---------------------------------------

        if (gamepad1.right_bumper) {
            if (tagDistanceM < 170){
                targetRPM = 3200;
            } else if (tagDistanceM < 220){
                targetRPM = 3500;
            } else {
                targetRPM = 3700;
            }
        } else {
            targetRPM = 0.0;
        }

        maxRPM = 6000.0;
        ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
        maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;

        flyMotor.setVelocity(ticksPerSecond);

        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

//        xNow = gamepad1.x;
//        if (xNow && !xLast) {
//            isUp = !isUp;
//        }
//        xLast = xNow;

        if (gamepad1.b) {
            keeperServo.setPosition(FLICKER_PRESSED_POS);
        } else {
            keeperServo.setPosition(FLICKER_DEFAULT_POS);
        }

        // --------------------------------------------------------------------------

        // PD tuning inputs (Gamepad 2)
        if (gamepad2.bWasPressed()) {stepIndex = (stepIndex + 1) % stepSizes.length;}
        if (gamepad2.dpadLeftWasPressed()) {kP -= stepSizes[stepIndex];}
        if (gamepad2.dpadRightWasPressed()) {kP += stepSizes[stepIndex];}
        if (gamepad2.dpadUpWasPressed()) {kD += stepSizes[stepIndex];}
        if (gamepad2.dpadDownWasPressed()) {kD -= stepSizes[stepIndex];}
        if (gamepad2.y) {kF += stepSizes[stepIndex];}
        if (gamepad2.a) {kF -= stepSizes[stepIndex];}

        // Telemetry
        double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Heading (Deg)", "%.2f", headingDeg);
        telemetry.addLine("----------------------");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("TPS", ticksPerSecond);
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", kD);
        telemetry.addData("Tuning F", "%.4f", kF);
        telemetry.addData("Step Size", "%.4f (B)", stepSizes[stepIndex]);
        telemetry.update();
    }
    // --------------------------------------------------

    // ------------------ INIT METHODS ----------------------
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.init();

        // Initialize the IMU directly from hardwareMap for field-centric orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyMotor");
        keeperServo = hardwareMap.get(Servo.class, "keeperServo");

        flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initialized Successfully");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        currentTime = getRuntime();
        xLast = false;
    }

    public void hoodCalc(double aprTagDist) {
        final double SERVO_POS_INCREMENT = 0.12;
        double[] dist = {130.0, 160.0, 190.0, 210.0, 230.0};

        int bestIndex = 0;
        double smallestError = Math.abs(aprTagDist - dist[0]);

        for (int i = 1; i < dist.length; i++) {
            double error = Math.abs(aprTagDist - dist[i]);
            if (error < smallestError) {
                smallestError = error;
                bestIndex = i;
            }
        }

        telemetry.addData("Hood index: %d", bestIndex);
    }
}