package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "Far Blue", group = "Auto")
public class AutonBlueFar extends LinearOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private DcMotorEx intakeMotor, flyMotor;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private Servo hoodServo, keeperServo;

    private static final double FLICKER_DEFAULT_POS = 0.0;
    private static final double FLICKER_PRESSED_POS = 0.7;

    // ===== DRIVE CONSTANTS =====
    private static final double TICKS_PER_INCH = 41.8; // 537.6 / (π * 4.094)
    private static final double TRACK_WIDTH = 15.0;    // Measure center-to-center of wheels

    @Override
    public void runOpMode() {


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Drive motors
        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // shooter motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyMotor");

        // direction servos
        keeperServo = hardwareMap.get(Servo.class, "keeperServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Brake behavior for intake and fly motors to prevent accidental firing
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flyMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        keeperServo.setPosition(FLICKER_DEFAULT_POS);
        hoodServo.setPosition(0.66);

        aprilTagWebcam.init(hardwareMap, telemetry);

        // Shooter setup
        final double TICKS_PER_REV_FLY = 28.0;
        double targetRPM = 3700;
        double ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;

        flyMotor.setVelocityPIDFCoefficients(80, 0, 10, 12);

        while (!isStarted() && !isStopRequested()) {
            aprilTagWebcam.update();
            telemetry.addLine("AprilTag tracking...");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        flyMotor.setVelocity(ticksPerSecond);

        // ===== AUTO ROUTINE =====
        flyMotor.setVelocity(ticksPerSecond);
        sleep(300);
        // move backwards to shooting position
        driveDistance(drive, 0.8, 0, 2.5);

        // turn to be perpendicular to wall
        turnDegrees(drive, -35, 0.6);
        // move out


        //first round of fire
        fireAndLoad();
        fireAndLoad();
        fireAndLoad();

        flyMotor.setPower(0);

        turnDegrees(drive, 45, -0.6);

        driveDistance(drive, 0.8, 0, 10);


    }

    // ==============================
    // DRIVE METHODS
    // ==============================

    public void driveDistance(MecanumDrive drive, double xPower, double yPower, double inches) {

        int targetTicks = (int)(inches * TICKS_PER_INCH);
        resetDriveEncoders();

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPower, yPower), 0));

        while (opModeIsActive() &&
                Math.abs(frontLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(100);
    }

    public void strafeDistance(MecanumDrive drive, double inches, double power) {
        driveDistance(drive, 0, power, inches);
    }

    public void turnDegrees(MecanumDrive drive, double degrees, double power) {

        double turnCircumference = Math.PI * TRACK_WIDTH;
        double arcLength = turnCircumference * (Math.abs(degrees) / 360.0);
        int targetTicks = (int)(arcLength * TICKS_PER_INCH);

        resetDriveEncoders();

        double turnRate = Math.toRadians(120);

        if (degrees > 0) {
            drive.setDrivePowers(
                    new PoseVelocity2d(new Vector2d(0, 0), turnRate)
            );
        } else {
            drive.setDrivePowers(
                    new PoseVelocity2d(new Vector2d(0, 0), -turnRate)
            );
        }

        while (opModeIsActive() &&
                Math.abs(frontLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(150);
    }

    public void resetDriveEncoders() {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ==============================
    // SHOOTER
    // ==============================

    public void fireAndLoad() {
        intakeMotor.setPower(0.0);
        keeperServo.setPosition(FLICKER_DEFAULT_POS);
        sleep(200);
        keeperServo.setPosition(FLICKER_PRESSED_POS);
        sleep(600);
        keeperServo.setPosition(FLICKER_DEFAULT_POS);
        sleep(400);
        intakeMotor.setPower(-1.0);
        sleep(550);
        intakeMotor.setPower(0.0);
        sleep(100);
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
    }
}