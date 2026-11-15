package org.firstinspires.ftc.teamcode;

// Positioning related packages

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "LebronRed", group = "Drive")
public class MecanumHeleOP extends LinearOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private DcMotorEx intakeMotor;
    private DcMotorEx ringMotor;
    private DcMotorEx flyMotor;

    private Servo hoodServo;
    private Servo keeperServo;

    //private Servo liftServo;

    // Constants for gear ratios and conversions to encoder ticks
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double GEAR1 = 1 + (46.0 / 17.0);
    private static final double GEAR2 = 1 + (46.0 / 17.0);
    private static final double GEAR3 = 1 + (46.0 / 17.0);
    private static final double TURRET_STAGE_MULTIPLIER = 5.0;

    private final double ticksPerRotation = MOTOR_TICKS_PER_REV * GEAR1 * GEAR2 * GEAR3;
    private final double ticksPerTurretRotation = ticksPerRotation * TURRET_STAGE_MULTIPLIER;

    // --- NEW: turret angle limits ---
    private static final double MIN_TURRET_DEG = -60.0; // initial left
    private static final double MAX_TURRET_DEG = 90.0;  // max right

    private volatile int turretTargetTicks = 0;
    private boolean prevY = false;

    // Flicker servo positions
    private static final double FLICKER_DEFAULT_POS = 0.0;
    private static final double FLICKER_PRESSED_POS = 0.4;
    private static final double KEEPER_DEFAULT_POS = 0.5;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Initialize motors
        aprilTagWebcam.init(hardwareMap, telemetry);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        ringMotor = hardwareMap.get(DcMotorEx.class, "ringMotor");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyMotor");
        keeperServo = hardwareMap.get(Servo.class, "keeperServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        //liftServo = null;//hardwareMap.get(Servo.class, "liftServo");

        // Reset turret encoder and set mode
//        ringMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        ringMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        ringMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // --- NEW: Set initial turret position to 60° left ---
        turretTargetTicks = degToTicks(MIN_TURRET_DEG);

        // Set motor PIDF for velocity control
        PIDFCoefficients turretPID = new PIDFCoefficients(16, 0, 0, 14);
        //ringMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, turretPID);

        if (keeperServo != null) keeperServo.setPosition(KEEPER_DEFAULT_POS);

        double targetRPM = 3700.0;

        // Flywheel setup
        final double TICKS_PER_REV_FLY = 28.0;

        double maxRPM = 6000.0;
        double ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
        double maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;
        flyMotor.setVelocityPIDFCoefficients(80, 0, 10, 0); // Tune this for flywheel PID

        // --- turret PID velocity parameters ---
        final double turretKPVelocity = 5.0; // ticks/sec per tick of error (tune this!)
        final double turretToleranceDeg = 5.0;
        final int turretToleranceTicks = Math.max(1, (int) (ticksPerTurretRotation * turretToleranceDeg / 360.0));

        waitForStart();

        if (isStopRequested()) return;

        hoodServo.setPosition(0.0);
        //liftServo.setPosition(0.7);

        while (opModeIsActive() && !isStopRequested()) {

            // --- Flywheel speed control ---
            if (gamepad1.right_bumper) {
                targetRPM = 3700.0;
            } else if (gamepad1.left_bumper) {
                targetRPM = 5000.0;
            } else {
                targetRPM = 0.0;
            }
            maxRPM = 6000.0;
            ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
            maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;
            flyMotor.setVelocity(ticksPerSecond);

            // --- Drive controls ---
            double y = -gamepad1.left_stick_x * 1.1;
            double x = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            Pose2d pose = drive.localizer.getPose();
            Vector2d input = new Vector2d(
                    x * Math.cos(-pose.heading.toDouble()) - y * Math.sin(-pose.heading.toDouble()),
                    x * Math.sin(-pose.heading.toDouble()) + y * Math.cos(-pose.heading.toDouble())
            );

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input.x, input.y), rx));

            // --- AprilTag detection for turret targeting ---
//            aprilTagWebcam.update();
//            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
//
//            if (id24 != null) {
//                double tagYaw = id24.ftcPose.yaw;
//                double currTurretAngle = ringMotor.getCurrentPosition();
//                double targetTurretAngle = calculateTarget(tagYaw, currTurretAngle);
//                turretTargetTicks = (int) Math.round(targetTurretAngle);
//
//                telemetry.addData("Tag Yaw", tagYaw);
//                telemetry.addData("Requested Target (deg)", ticksToDeg(turretTargetTicks));
//                double dx = id24.ftcPose.x;
//                double dy = id24.ftcPose.y;
//                double dz = id24.ftcPose.z;
//                double tagDistanceM = Math.sqrt(dx*dx + dy*dy + dz*dz);
//                telemetry.addData("Tag Distance (cm)", String.format("%.2f", tagDistanceM ));
//
//                hoodCalc(tagDistanceM);
//            }
//
//            // --- Turret velocity PID controller ---
//            int currentTicks = ringMotor.getCurrentPosition();
//            int error = turretTargetTicks - currentTicks;
//
//            double targetVelocity = 0.0;
//            if (Math.abs(error) > turretToleranceTicks) {
//                targetVelocity = turretKPVelocity * error;
//                // Clamp velocity to motor limits
//                targetVelocity = Math.max(-maxTPS, Math.min(maxTPS, targetVelocity));
//            }
//
//            ringMotor.setVelocity(targetVelocity);

            // --- Servo controls ---
            if (gamepad1.b) keeperServo.setPosition(FLICKER_PRESSED_POS);
            else keeperServo.setPosition(FLICKER_DEFAULT_POS);

            // --- Gamepad turret manual override ---
//            moveTurretToAngleDegController(0.0); // example: set desired angle via Y button

            // --- Intake controls ---
            if (gamepad1.a) {
                intakeMotor.setPower(-1.0);
            } else if (gamepad1.y) {
                intakeMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

          /*  if (gamepad1.x) {
                liftServo.setPosition(0.0);
            } else {
                liftServo.setPosition(0.7);
            }*/

            // --- Telemetry ---
//            telemetry.addData("Target Turret Angle (deg)", ticksToDeg(turretTargetTicks));
//            telemetry.addData("Current Turret Angle (deg)", ticksToDeg(currentTicks));
//            telemetry.addData("Turret Error (ticks)", error);
//            telemetry.addData("Target Velocity (TPS)", targetVelocity);
//
//            telemetry.update();
        }
    }

    // --- HELPER METHODS ---

    public void moveTurretToAngleDegController(double desiredAngleDeg) {
        if (ringMotor == null) {
            prevY = gamepad1.y;
            return;
        }

        if (ringMotor.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            ringMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        boolean currY = gamepad1.y;

        // Rising edge trigger
        if (currY && !prevY) {
            // Clamp desired angle to turret limits
            double clampedDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, desiredAngleDeg));

            // Convert clamped angle to motor ticks
            turretTargetTicks = degToTicks(clampedDeg);
        }

        prevY = currY;
    }

    public double calculateTarget(double tagYaw, double currAngleTicks) {
        double normalizeCurrAngle = currAngleTicks / ticksPerTurretRotation;
        double tagFraction = (tagYaw + 14.0) / 360.0;
        double targetAngle = normalizeCurrAngle + tagFraction;

        // --- Clamp to allowed turret range ---
        if (targetAngle * 360.0 > MAX_TURRET_DEG) targetAngle = MAX_TURRET_DEG / 360.0;
        if (targetAngle * 360.0 < MIN_TURRET_DEG) targetAngle = MIN_TURRET_DEG / 360.0;

        return targetAngle * ticksPerTurretRotation;
    }

    public void hoodCalc(double aprTagDist) {
        final double SERVO_POS_INCREMENT = 0.12;

        double[] dist = {224.65, 272.0, 306.25, 361.7, 352.60};

        int bestIndex = 0;
        double smallestError = Math.abs(aprTagDist - dist[0]);

        for (int i = 1; i < dist.length; i++) {
            double error = Math.abs(aprTagDist - dist[i]);
            if (error < smallestError) {
                smallestError = error;
                bestIndex = i;
            }
        }

        hoodServo.setPosition(SERVO_POS_INCREMENT * bestIndex);
        telemetry.addData("Hood index: %d", bestIndex);
    }

    // --- NEW: Angle <-> ticks helpers ---
    private int degToTicks(double deg) {
        return (int) Math.round((deg / 360.0) * ticksPerTurretRotation);
    }

    private double ticksToDeg(int ticks) {
        return (ticks / ticksPerTurretRotation) * 360.0;
    }
}