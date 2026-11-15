package org.firstinspires.ftc.teamcode;

// Positioning related packages

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Next Gen Red", group = "Auto")
public class AutoRed extends LinearOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private DcMotorEx intakeMotor;
    private DcMotorEx flyMotor;

    private Servo hoodServo;
    private Servo keeperServo;

    // Flicker servo positions
    private static final double FLICKER_DEFAULT_POS = 0.0;
    private static final double FLICKER_PRESSED_POS = 0.4;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Initialize hardware
        aprilTagWebcam.init(hardwareMap, telemetry);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyMotor");

        // Brake behaviour for intake motor to prevent unwanted movement when power is zero
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flyMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        keeperServo = hardwareMap.get(Servo.class, "keeperServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");


        keeperServo.setPosition(FLICKER_DEFAULT_POS);
        hoodServo.setPosition(0.0);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // Flywheel PID (unchanged)
        final double TICKS_PER_REV_FLY = 28.0;
        flyMotor.setVelocityPIDFCoefficients(80, 0, 10, 0);


        // --- INIT LOOP: ensure camera is tracking before start ---
        while (!isStarted() && !isStopRequested()) {
            aprilTagWebcam.update();
            telemetry.addLine("AprilTag tracking...");
            telemetry.update();
        }

        double targetRPM = 3300.0;
        double maxRPM = 6000.0;
        double ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
        double maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;
        flyMotor.setVelocity(ticksPerSecond);

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive() && !isStopRequested()) {

            // ---------------- INITIAL STOP ----------------
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

            // ---------------- APRILTAG AIM ----------------
            aprilTagWebcam.update();
            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(20);

            if (id24 != null) {
                double dx = id24.ftcPose.x;
                double dy = id24.ftcPose.y;
                double dz = id24.ftcPose.z;
                double tagDistanceM = Math.sqrt(dx * dx + dy * dy + dz * dz);
                hoodCalc(tagDistanceM);
            }

            keeperServo.setPosition(FLICKER_DEFAULT_POS);

            // ================= DRIVE BACK =================
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(drive.localizer.getPose().position.x - 24) // was -0.8 for ~1.08s
                            .build()
            );

            // ================= FIRE 3 =================
            fireAndLoad();
            fireAndLoad();
            fireAndLoad();

            // ================= TURN LEFT =================
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(90)) // matches original positive turn
                            .build()
            );

            // ================= STRAFE LEFT =================
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(
                                    drive.localizer.getPose().position.x,
                                    drive.localizer.getPose().position.y - 14
                            ))
                            .build()
            );

            // ================= INTAKE FIRST SPIKE =================
            intakeMotor.setPower(-1.0);

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(drive.localizer.getPose().position.x + 20)
                            .build()
            );

            // ================= ANGLE TO BASKET =================
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(drive.localizer.getPose().position.x - 12)
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(
                                    drive.localizer.getPose().position.x,
                                    drive.localizer.getPose().position.y + 16
                            ))
                            .build()
            );

            intakeMotor.setPower(0.0);

            // ================= SPIN UP SHOOTER =================
            flyMotor.setVelocity(ticksPerSecond);

            // ================= TURN RIGHT =================
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(-90))
                            .build()
            );

            // ================= MOVE TO GOAL =================
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(drive.localizer.getPose().position.x + 4)
                            .build()
            );

            // ================= FIRE 3 =================
            fireAndLoad();
            fireAndLoad();
            fireAndLoad();

            // ================= TURN TO SECOND SPIKE =================
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(90))
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(
                                    drive.localizer.getPose().position.x,
                                    drive.localizer.getPose().position.y - 8
                            ))
                            .build()
            );

            // ================= SECOND SPIKE INTAKE =================
            intakeMotor.setPower(-1.0);

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(drive.localizer.getPose().position.x + 20)
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(drive.localizer.getPose().position.x - 12)
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(
                                    drive.localizer.getPose().position.x,
                                    drive.localizer.getPose().position.y + 10
                            ))
                            .build()
            );

            intakeMotor.setPower(0.0);

            // ================= FINAL ALIGN =================
            flyMotor.setVelocity(ticksPerSecond);

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(-90))
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(drive.localizer.getPose().position.x - 6)
                            .build()
            );

            // ================= FINAL FIRE =================
            fireAndLoad();
            fireAndLoad();
            fireAndLoad();
        }
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

    public void fireAndLoad() {
        intakeMotor.setPower(0.0); // intake off
        keeperServo.setPosition(FLICKER_DEFAULT_POS); // pressed
        sleep(200); // keep pressed for 1 second
        keeperServo.setPosition(FLICKER_PRESSED_POS); // release
        sleep(500); // pause for 0.5 seconds
        keeperServo.setPosition(FLICKER_DEFAULT_POS); // pressed
        sleep(200); // keep pressed for 0.5 seconds
        intakeMotor.setPower(-1.0); // intake on
        sleep(550); // run for 0.5 seconds
        intakeMotor.setPower(0.0); // intake off
        sleep(100);
    }
}
