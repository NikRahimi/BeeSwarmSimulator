package org.firstinspires.ftc.teamcode;

// Positioning related packages

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "mve forward", group = "Auto")
public class MoveForwardAuton extends LinearOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private DcMotorEx intakeMotor;
    private DcMotorEx ringMotor;
    private DcMotorEx flyMotor;

    private Servo hoodServo;
    private Servo keeperServo;

    // Constants for gear ratios and conversions to encoder ticks
//    private static final double MOTOR_TICKS_PER_REV = 28.0;
//    private static final double GEAR1 = 1 + (46.0 / 17.0);
//    private static final double GEAR2 = 1 + (46.0 / 11.0);
//    private static final double TURRET_STAGE_MULTIPLIER = 5.0;
//
//    private final double ticksPerRotation = MOTOR_TICKS_PER_REV * GEAR1 * GEAR2;
//    private final double ticksPerTurretRotation = ticksPerRotation * TURRET_STAGE_MULTIPLIER;
//
//    // Turret angle limits
//    private static final double MIN_TURRET_DEG = -60.0;
//    private static final double MAX_TURRET_DEG = 90.0;

//    private volatile int turretTargetTicks = 0;

    // Flicker servo positions
    private static final double FLICKER_DEFAULT_POS = 0.0;
    private static final double FLICKER_PRESSED_POS = 0.4;
    private static final double KEEPER_DEFAULT_POS = 0.5;

    double sequenceStartTime = getRuntime(); // remember when this sequence started
    boolean keeperDone = false;
    boolean intakeDone = false;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Initialize hardware
        aprilTagWebcam.init(hardwareMap, telemetry);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        ringMotor = hardwareMap.get(DcMotorEx.class, "ringMotor");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyMotor");

        keeperServo = hardwareMap.get(Servo.class, "keeperServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Turret setup
        ringMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ringMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ringMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients turretPID = new PIDFCoefficients(16, 0, 0, 14);
        ringMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, turretPID);

        keeperServo.setPosition(FLICKER_DEFAULT_POS);
        hoodServo.setPosition(0.0);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // Flywheel PID (unchanged)
        final double TICKS_PER_REV_FLY = 28.0;
        flyMotor.setVelocityPIDFCoefficients(80, 0, 10, 0);

        // Turret PID parameters (unchanged)
//        final double turretKPVelocity = 5.0;
//        final double turretToleranceDeg = 5.0;
//        final int turretToleranceTicks =
//                Math.max(1, (int) (ticksPerTurretRotation * turretToleranceDeg / 360.0));

        // --- INIT LOOP: ensure camera is tracking before start ---
        while (!isStarted() && !isStopRequested()) {
            aprilTagWebcam.update();
            telemetry.addLine("AprilTag tracking...");
            telemetry.update();
        }

        double targetRPM = 3700.0;
        double maxRPM = 6000.0;
        double ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
        double maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;
        flyMotor.setVelocity(ticksPerSecond);

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive() && !isStopRequested()) {
            // Stop drivetrain (same structure, just no input)
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));



//            int targetTicks = degToTicks(-90); // e.g., 0° turret
//
//            ringMotor.setTargetPosition(targetTicks);
//            ringMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            ringMotor.setVelocity(500);

            keeperServo.setPosition((FLICKER_DEFAULT_POS));

// --- Step 1: Drive backwards for 2 seconds ---
            hoodServo.setPosition(0.6);
            sleep(1000); // pause for 1 second to let hood catch up
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 5)); // stop
            sleep(125); // pause for 1 second to let turret catch up
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(500); // pause for 1 second to let hood catch up
          for (int i = 0; i < 3; i++) {
              keeperServo.setPosition(FLICKER_PRESSED_POS);
              sleep(1000); // pause for 1 second to let the shot go
              keeperServo.setPosition((FLICKER_DEFAULT_POS));
              sleep(1000); // pause for 1 second to let the servo reset
              intakeMotor.setPower(-0.9);
              sleep(1000); // pause for 1 second to let the servo reset
          }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.4, 0), 0));
            sleep(1500); // pause for 2.1 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sleep(500);
            hoodServo.setPosition(0.0);
            sleep(500);

            // --- Telemetry ---
            telemetry.update();
        }
    }

    // ---------- Helper methods (UNCHANGED) ----------

//    private int degToTicks(double deg) {
//        return (int) Math.round((deg / 360.0) * ticksPerTurretRotation);
//    }
//
//    private double ticksToDeg(int ticks) {
//        return (ticks / ticksPerTurretRotation) * 360.0;
//    }
//
//    public double calculateTarget(double tagYaw, double currAngleTicks) {
//        double normalizeCurrAngle = currAngleTicks / ticksPerTurretRotation;
//        double tagFraction = (tagYaw + 14.0) / 360.0;
//        double targetAngle = normalizeCurrAngle + tagFraction;
//
//        if (targetAngle * 360.0 > MAX_TURRET_DEG) targetAngle = MAX_TURRET_DEG / 360.0;
//        if (targetAngle * 360.0 < MIN_TURRET_DEG) targetAngle = MIN_TURRET_DEG / 360.0;
//
//        return targetAngle * ticksPerTurretRotation;
    //    }

//    public void hoodCalc(double aprTagDist) {
//        final double SERVO_POS_INCREMENT = 0.12;
//
//        double[] dist = {224.65, 272.0, 306.25, 361.7, 352.60};
//
//        int bestIndex = 0;
//        double smallestError = Math.abs(aprTagDist - dist[0]);
//
//        for (int i = 1; i < dist.length; i++) {
//            double error = Math.abs(aprTagDist - dist[i]);
//            if (error < smallestError) {
//                smallestError = error;
//                bestIndex = i;
//            }
//        }
//
//        hoodServo.setPosition(SERVO_POS_INCREMENT * bestIndex);
    }
