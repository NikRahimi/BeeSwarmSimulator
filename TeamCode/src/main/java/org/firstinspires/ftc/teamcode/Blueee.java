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

    double[][] lookup = {{1790.0, 48.0}, {1760.0, 43.0}, {1690.0, 38.0}, {1640.0, 33.0}, {1630.0, 28.0}, {1640.0, 23.0}};

    // Flywheel variables
    double targetRPM = 0.0;
    final double TICKS_PER_REV_FLY = 28.0;
    double maxRPM = 6000.0;
    double ticksPerSecond = 0.0;

    // Flywheel RPM you set from the controller (gamepad 1 D-pad). The right bumper runs the flywheel at this RPM.
    double flywheelSetRPM = 1830.0;

    // Servo toggle variables for flower servos
    boolean flowerState = false;
    boolean xLastState = false;

    // Turret Target Toggle Variables (Gamepad 1 Y Button)
    boolean targetToggle = false; // false = Target Set A, true = Target Set B
    boolean yLastState = false;

    // Gamepad 2 bumper edge detection (feedforward tuning)
    boolean lbLast = false;
    boolean rbLast = false;
    boolean backLast = false;

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
    double turretKFF = 1.2;                  // Feedforward (power per rad/s of robot spin). ~1/(turret free speed in rad/s); tune on gamepad 2 bumpers
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

        // (Flywheel control moved below the turret target math, since its RPM now depends on distance)

        // Flower Servo Toggle Logic using Gamepad 1 X Button
        boolean xPressedNow = gamepad1.x;
        if (xPressedNow && !xLastState) {
            flowerState = !flowerState;
        }
        xLastState = xPressedNow;

        if (flowerState) {
            flowerServo.setPosition(0.3);
            flowerServo2.setPosition(0.3);
        } else {
            flowerServo.setPosition(0.1);
            flowerServo2.setPosition(0.1);
        }

        // Intake control on Gamepad 1 A Button
        if (gamepad1.a && gamepad1.b) {
            intakeMotor.setPower(-0.77);
        } else if (gamepad1.a) {
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

        // Straight-line distance from the robot to the active target (inches)
        double distanceToTargetIn = Math.hypot(dx, dy);

        // Flywheel RPM adjust (gamepad 1 D-pad): up/down = +/-50 RPM, right/left = +/-10 RPM
        if (gamepad1.dpadUpWasPressed()) {flywheelSetRPM += 50.0;}
        if (gamepad1.dpadDownWasPressed()) {flywheelSetRPM -= 50.0;}
        if (gamepad1.dpadRightWasPressed()) {flywheelSetRPM += 10.0;}
        if (gamepad1.dpadLeftWasPressed()) {flywheelSetRPM -= 10.0;}
        flywheelSetRPM = Range.clip(flywheelSetRPM, 0.0, maxRPM);

        // Flywheel: hold right bumper to spin up to the set RPM
        if (gamepad1.right_bumper) {
            //targetRPM = flywheelSetRPM;
            targetRPM = bestRPM(lookup, distanceToTargetIn);
        } else {
            targetRPM = 0.0;
        }

        ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;

        // Drive flywheels via velocity PID
        flyMotor.setVelocity(ticksPerSecond);
        flyMotor2.setVelocity(ticksPerSecond);

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

        // FEEDFORWARD: the robot spinning at omega (CCW positive) makes the target appear to rotate
        // by -omega relative to the chassis, so the turret must counter-rotate at that speed.
        // Off in test mode (fixed angle).
        double robotOmegaRadPerSec = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        double targetVelRadPerSec = testMode ? 0.0 : -robotOmegaRadPerSec;
        boolean targetMoving = Math.abs(targetVelRadPerSec) > 0.15; // ~9 deg/s

        double computedTurretPower = 0.0;
        // Hold the deadband only when the target is still; while it's moving keep tracking smoothly
        if (Math.abs(angleErrorRad) > deadbandRad || targetMoving) {
            // P pushes toward the target, D damps the speed MISMATCH, FF supplies the counter-rotation
            double rawPower = (angleErrorRad * turretKP)
                    + (targetVelRadPerSec - turretVelRadPerSec) * turretKD
                    + (targetVelRadPerSec * turretKFF);
            computedTurretPower = Range.clip(rawPower, -turretMaxPower, turretMaxPower);

            // Small friction kick, only outside the deadband and only if pushing toward the target
            if (Math.abs(angleErrorRad) > deadbandRad
                    && Math.abs(computedTurretPower) < turretMinPower
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
        packet.put("Flywheel Set RPM", flywheelSetRPM);
        packet.put("Target RPM", targetRPM);
        packet.put("Distance to Target (in)", distanceToTargetIn);
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

        // Feedforward tuning: gamepad 2 left bumper = kFF down, right bumper = kFF up
        boolean lbNow = gamepad2.left_bumper;
        boolean rbNow = gamepad2.right_bumper;
        if (lbNow && !lbLast) {turretKFF -= stepSizes[stepIndex];}
        if (rbNow && !rbLast) {turretKFF += stepSizes[stepIndex];}
        lbLast = lbNow;
        rbLast = rbNow;

        // RE-ZERO: if the turret slipped or got bumped, physically center it, then press gamepad 2 BACK
        // to reset the encoder so this spot becomes 0 degrees again.
        boolean backNow = gamepad2.back;
        if (backNow && !backLast) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        backLast = backNow;

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
        telemetry.addData("Turret kFF", "%.4f", turretKFF);
        telemetry.addData("Robot spin (deg/s)", "%.1f", Math.toDegrees(robotOmegaRadPerSec));
        telemetry.addData("Tuning step", stepSizes[stepIndex]);
        telemetry.addData("Turret Test Mode", testMode ? "ON (fixed angle)" : "off (tracking)");
        telemetry.addData("At Soft Limit", hitSoftLimit);
        telemetry.addData("Flywheel Set RPM", "%.0f", flywheelSetRPM);
        telemetry.addData("Flywheel Target RPM (live)", "%.0f", targetRPM);
        telemetry.addData("Distance to Target (in)", "%.1f", distanceToTargetIn);
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

    private double bestRPM(double[][] table, double distance) {
        double bestRpm = 0.0;
        double bestDiff = Double.MAX_VALUE;

        for (double[] row : table) {
            double diff = Math.abs(row[1] - distance); // row[0] = rpm, row[1] = distance
            if (diff < bestDiff) {
                bestDiff = diff;
                bestRpm = row[0];
            }
        }
        return bestRpm;
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
        flyMotor.setVelocityPIDFCoefficients(280, 0, 0, 35);

        flyMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyMotor2.setVelocityPIDFCoefficients(280, 0, 0, 35);

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
        lbLast = false;
        rbLast = false;
        backLast = false;
    }
}