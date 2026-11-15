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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Lebron Auto RED", group = "Auto")
public class AutonomousRED extends LinearOpMode {

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

        double targetRPM = 3200.0;
        double maxRPM = 6000.0;
        double ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
        double maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;
        flyMotor.setVelocity(ticksPerSecond);

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive() && !isStopRequested()) {
            // Stop drivetrain (same structure, just no input)
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

            // --- AprilTag detection for turret targeting ---
            aprilTagWebcam.update();
            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(20);

            if (id24 != null) {

                double dx = id24.ftcPose.x;
                double dy = id24.ftcPose.y;
                double dz = id24.ftcPose.z;
                double tagDistanceM = Math.sqrt(dx * dx + dy * dy + dz * dz);

                hoodCalc(tagDistanceM);
            }

            keeperServo.setPosition((FLICKER_DEFAULT_POS));

// --- Step 1: Drive backwards for 2 seconds ---
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.8, 0), 0));
            sleep(1080);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(500);

            // 1 round of fire //
            fireAndLoad();
            fireAndLoad();
            fireAndLoad();

            //Turn function//
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 90));
            sleep(270); // pause for 2 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(500);

            // strafe to the first spike mark //
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -0.8), 0));
            sleep(660); // pause for 2 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop

            //turn off shooter
            flyMotor.setPower(0);
            sleep(200);

            // first spike mark //
            intakeMotor.setPower(-1.0);// intake on
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.4, 0), 0));
            sleep(1450); // pause for 2.1 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(200);

            // angle with basket//
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.7, 0), 0));
            sleep(600); // pause for 2.1 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0.8), 0));
            sleep(760); // pause for 2 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100);
            // end the intake after the first spike mark
            intakeMotor.setPower(0.0); // intake off
            sleep(200);

            flyMotor.setVelocity(ticksPerSecond);
            sleep(100);

            //turn function//
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), -90));
            sleep(230); // pause for 2 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100); // pause for 2 seconds

            // move closer to goal//
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1.0, 0), 0));
            sleep(25); // pause for 2.1 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100);

            // 2nd round of fire //
            fireAndLoad();
            fireAndLoad();
            fireAndLoad();

            flyMotor.setPower(0);
            sleep(50);
            // reach the 2nd spike mark //
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 90));
            sleep(250); // pause for 2 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100); // pause for 2 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -0.8), 0));
            sleep(175);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop

            // 2nd spike mark intake sequence//
            intakeMotor.setPower(-1.0);// intake on
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.4, 0), 0));
            sleep(1450); // pause for 2.1 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.8, 0), 0));
            sleep(600); // pause for 2.1 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100);

            // strafe to the original pos//
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0.8), 0));
            sleep(150); // pause for 2 seconds
            // end the intake after the first spike mark
            intakeMotor.setPower(0.0); // intake off
            sleep(200);

            //end intake and initialize the fly wheel//
            flyMotor.setVelocity(ticksPerSecond);
            sleep(100);

            //turn to align with the net //
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), -90));
            sleep(220); // pause for 2 seconds
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // stop
            sleep(100); // pause for 2 seconds

            // move backwards to create net //
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.5, 0), 0));
            sleep(175);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sleep(100);

            // 3rd round of fire //
            fireAndLoad();
            fireAndLoad();
            fireAndLoad();

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -1.0), 0));
            sleep(500); // pause for 2 seconds

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
