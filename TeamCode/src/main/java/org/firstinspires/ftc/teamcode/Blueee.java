package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "BLEW", group = "Competition")
public class Blueee extends OpMode {

    // -------------------- VARIABLES --------------------

    MecanumDrive drive;
    IMU imu;

    // Hardware init
    private DcMotorEx intakeMotor;
    private DcMotorEx flyMotor;
    private DcMotorEx flyMotor2;
    private DcMotorEx turretMotor;
    private Servo keeperServo;
    private Servo flowerServo;
    private Servo flowerServo2;

    // PIDF tuning variables
    double kP = 0.058;
    double angleTolerance = 2;
    double kD = 0.0;
    double kF = 0.4;

    // Flywheel variables
    double targetRPM = 0.0;
    final double TICKS_PER_REV_FLY = 28.0;
    double maxRPM = 6000.0;
    double ticksPerSecond = 0.0;

    // Servo toggle variables for flower servos
    boolean flowerState = false;
    boolean xLastState = false;

    // Turret Target Toggle Variables (Gamepad 1 Y Button)
    boolean targetToggle = false; // false = Target Set A, true = Target Set B
    boolean yLastState = false;

    // Field Target Coordinates (Relative to start tile: Column 1, 3 tiles down = (0,0))
    // Target Set A (e.g., Primary Bucket)
    double targetAX = 36.0;
    double targetAY = 12.0;

    // Target Set B (e.g., Alternate/Flipped Side Bucket)
    double targetBX = 36.0;
    double targetBY = -12.0;

    // Turret physical & tracking constants (adjust to match your robot)
    final double TURRET_MOTOR_TICKS_PER_REV = 28.0; // 6000 rpm motor, encoder ticks per motor shaft rev
    final double TURRET_GEAR_RATIO = 60.0;    // Motor rotations per 1 turret rotation (measured: ~420 ticks at 90 deg => 1680 ticks/rev => 60)
    final double TURRET_MAX_SAFE_DEG = 135.0; // Max safe rotation each way from center (cable limit)
    final double TURRET_DEADBAND_DEG = 1.5;   // Stop driving once within this error window (one encoder tick is ~0.21 deg now)
    double turretZeroOffsetRad = 0.0;         // Offset if turret "0" isn't aligned with robot 0
    double turretKP = 0.80;                   // Proportional gain (power per radian of error)
    double turretKD = 0.02;                   // Damping gain (power per rad/s of turret speed)
    double turretMaxPower = 0.60;             // Max safe motor power
    double turretMinPower = 0.08;             // Small kick to overcome friction on the 60:1 reduction (0 to disable)

    // Driving setup
    double forward, strafe, rotate;

    // Controller-customized PIDF tuning steps
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    // ---------------------------------------------------

    @Override
    public void loop() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Keep Road Runner updating its internal tracking
        drive.updatePoseEstimate();

        // Retrieve current pose from odometry localization
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

        ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;

        // Drive flywheels via velocity PID
        flyMotor.setVelocity(ticksPerSecond);
        flyMotor2.setVelocity(ticksPerSecond);

        // Flower Servo Toggle Logic using Gamepad 1 X Button
        boolean xPressedNow = gamepad1.x;
        if (xPressedNow && !xLastState) {
            flowerState = !flowerState;
        }
        xLastState = xPressedNow;

        if (flowerState) {
            flowerServo.setPosition(0.75);
            flowerServo2.setPosition(0.75);
        } else {
            flowerServo.setPosition(0.1);
            flowerServo2.setPosition(0.1);
        }

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

        // ---------------------- PERMANENT TURRET TRACKING & SOFT LIMITS ----------------------

        // Toggle target coordinates using Gamepad 1 Y Button
        boolean yPressedNow = gamepad1.y;
        if (yPressedNow && !yLastState) {
            targetToggle = !targetToggle;
        }
        yLastState = yPressedNow;

        // Pick active target coordinates based on toggle state
        double activeTargetX = targetToggle ? targetBX : targetAX;
        double activeTargetY = targetToggle ? targetBY : targetAY;

        // Current robot pose from Road Runner odometry
        double robotX = currentPose.position.x;
        double robotY = currentPose.position.y;
        double robotHeadingRad = currentPose.heading.toDouble();

        // Vector from robot to target bucket in field coordinates
        double dx = activeTargetX - robotX;
        double dy = activeTargetY - robotY;
        double targetFieldAngleRad = Math.atan2(dy, dx);

        // Where the turret ACTUALLY is right now (radians, relative to robot chassis)
        double turretAngleRad = turretMotor.getCurrentPosition()
                / (TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO) * 2.0 * Math.PI;

        // Where we WANT the turret (radians, relative to robot chassis)
        double desiredTurretAngleRad = normalizeAngleRad(
                (targetFieldAngleRad - robotHeadingRad) - turretZeroOffsetRad
        );

        // TEST MODE: hold gamepad 2 A / X / Y to send the turret to a fixed angle (0 / 45 / 90 deg),
        // ignoring odometry. Use this to tune kP/kD without worrying about where the target is.
        boolean testMode = gamepad2.a || gamepad2.x || gamepad2.y;
        if (testMode) {
            double testDeg = gamepad2.a ? 0.0 : (gamepad2.x ? 45.0 : 90.0);
            desiredTurretAngleRad = Math.toRadians(testDeg);
        }

        // Enforce physical cable soft limits (e.g., +/- 135 degrees)
        double maxSafeAngleRad = Math.toRadians(TURRET_MAX_SAFE_DEG);
        boolean hitSoftLimit = Math.abs(desiredTurretAngleRad) > maxSafeAngleRad;
        double clampedTurretAngleRad = Range.clip(desiredTurretAngleRad, -maxSafeAngleRad, maxSafeAngleRad);

        // REAL error = where we want to be - where we are
        double angleErrorRad = clampedTurretAngleRad - turretAngleRad;
        double deadbandRad = Math.toRadians(TURRET_DEADBAND_DEG);

        // Turret angular velocity (rad/s) from the encoder, used as the D (damping) term
        double turretVelRadPerSec = turretMotor.getVelocity()
                / (TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO) * 2.0 * Math.PI;

        double computedTurretPower = 0.0;
        if (Math.abs(angleErrorRad) > deadbandRad) {
            // P pushes toward the target, D brakes against speed so it doesn't overshoot
            double rawPower = (angleErrorRad * turretKP) - (turretVelRadPerSec * turretKD);
            computedTurretPower = Range.clip(rawPower, -turretMaxPower, turretMaxPower);

            // Small friction kick, only if we're pushing toward the target
            if (Math.abs(computedTurretPower) < turretMinPower
                    && Math.signum(computedTurretPower) == Math.signum(angleErrorRad)) {
                computedTurretPower = Math.signum(angleErrorRad) * turretMinPower;
            }
        }

        // Hard safety: never drive further out if already at/past the cable limit
        if (turretAngleRad >= maxSafeAngleRad && computedTurretPower > 0) {
            computedTurretPower = 0.0;
        }
        if (turretAngleRad <= -maxSafeAngleRad && computedTurretPower < 0) {
            computedTurretPower = 0.0;
        }

        turretMotor.setPower(computedTurretPower);
        // -------------------------------------------------------------------------------------

        // FTC Dashboard Telemetry Streaming
        double actualRPM1 = (flyMotor.getVelocity() * 60.0) / TICKS_PER_REV_FLY;
        double actualRPM2 = (flyMotor2.getVelocity() * 60.0) / TICKS_PER_REV_FLY;
        packet.put("Target RPM", targetRPM);
        packet.put("Actual RPM (Motor 1)", actualRPM1);
        packet.put("Actual RPM (Motor 2)", actualRPM2);
        packet.put("Turret Target Set", targetToggle ? "Target B" : "Target A");
        packet.put("Turret Desired (Deg)", Math.toDegrees(clampedTurretAngleRad));
        packet.put("Turret Actual (Deg)", Math.toDegrees(turretAngleRad));
        packet.put("Turret Error (Deg)", Math.toDegrees(angleErrorRad));
        packet.put("Turret Power", computedTurretPower);
        packet.put("At Soft Limit", hitSoftLimit);
        dashboard.sendTelemetryPacket(packet);

        // PD tuning inputs (Gamepad 2)
        if (gamepad2.bWasPressed()) {stepIndex = (stepIndex + 1) % stepSizes.length;}
        if (gamepad2.dpadLeftWasPressed()) {turretKP -= stepSizes[stepIndex];}
        if (gamepad2.dpadRightWasPressed()) {turretKP += stepSizes[stepIndex];}
        if (gamepad2.dpadUpWasPressed()) {turretKD += stepSizes[stepIndex];}
        if (gamepad2.dpadDownWasPressed()) {turretKD -= stepSizes[stepIndex];}

        // Driver Station Telemetry
        double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Heading (Deg)", "%.2f", headingDeg);
        telemetry.addLine("----------------------");
        telemetry.addData("Turret Goal", targetToggle ? "Set B (X:36, Y:-12)" : "Set A (X:36, Y:12)");
        telemetry.addData("Turret Desired (Deg)", "%.1f", Math.toDegrees(clampedTurretAngleRad));
        telemetry.addData("Turret Actual (Deg)", "%.1f", Math.toDegrees(turretAngleRad));
        telemetry.addData("Turret Error (Deg)", "%.2f", Math.toDegrees(angleErrorRad));
        telemetry.addData("Turret Power", "%.2f", computedTurretPower);
        telemetry.addData("Turret kP / kD", "%.4f / %.4f", turretKP, turretKD);
        telemetry.addData("Tuning step", stepSizes[stepIndex]);
        telemetry.addData("Turret Test Mode", testMode ? "ON (fixed angle)" : "off (tracking)");
        telemetry.addData("At Soft Limit", hitSoftLimit);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flower State", flowerState ? "Active" : "Home");
        telemetry.addData("RR X (Inches)", "%.2f", currentPose.position.x);
        telemetry.addData("RR Y (Inches)", "%.2f", currentPose.position.y);
        telemetry.addData("RR Heading (Deg)", "%.2f", Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.update();
    }

    private double normalizeAngleRad(double angleRad) {
        while (angleRad > Math.PI) {
            angleRad -= 2.0 * Math.PI;
        }
        while (angleRad < -Math.PI) {
            angleRad += 2.0 * Math.PI;
        }
        return angleRad;
    }

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
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        // If the turret tracks the wrong way, uncomment this line (must be BEFORE the encoder reset):
        // turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Zero the encoder here, so the turret MUST be physically centered when you press INIT.
        // RUN_WITHOUT_ENCODER still reports position; it just means we drive with raw power.
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Velocity PID configuration for flywheels
        flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyMotor.setVelocityPIDFCoefficients(60, 2, 0, 11.7);

        flyMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyMotor2.setVelocityPIDFCoefficients(60, 2, 0, 11.7);

        flyMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flowerServo2.setDirection(Servo.Direction.REVERSE);

        flowerServo.setPosition(0.1);
        flowerServo2.setPosition(0.1);

        telemetry.addLine("Initialized Successfully");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        xLastState = false;
        yLastState = false;
    }
}