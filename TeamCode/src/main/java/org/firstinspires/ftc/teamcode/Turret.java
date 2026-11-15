package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
public class Turret extends OpMode {
    private AprilTagWebcam aprTag = new AprilTagWebcam();
    private TurretTest ringMotor = new TurretTest();
    double[] step = {0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 2;




    // -------------------- VARIABLES --------------------

    // Constructors
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    MecanumDrive drive;

    // Hardware init
    private DcMotorEx intakeMotor;
    private DcMotorEx flyMotor;

    private Servo hoodServo;
    private Servo keeperServo;

    //private Servo liftServo;

    // AprilTag ID number
    // 20 -> blue, 24 -> red
    int tagNum = 24
            ;

    // PIDF tuning variables
    double kP = 0.058;
    double angleTolerance = 2;
    double kD = 0.0;
    double kF = 0.4
            ;

    // Miscellaneous PIDF variables
    boolean alignActiveLastLoop = false;
    double deadband = 0.01; // Threshold to floor a rotation value deemed too small, stops unnecessary wheel movement
    double error = 0;
    double lastError = 0;
    double goalX = 0; // Offset goal by some degree
    double currentTime = 0;
    double lastTime = 0;

    // Flywheel variables
    double targetRPM = 3700.0;

    // Flywheel setup
    final double TICKS_PER_REV_FLY = 28.0;
    double maxRPM = 6000.0;
    double ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
    double maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;

    // Servo positions
    private static final double FLICKER_DEFAULT_POS = 0.0;
    private static final double FLICKER_PRESSED_POS = 0.4;
    double dx, dy, dz, tagDistanceM;
    boolean xNow;
    boolean xLast = false;
    boolean isUp = false;

    // Driving setup
    double forward,strafe, rotate;




    @Override
    public void init() {
        aprTag.init(hardwareMap, telemetry);
        ringMotor.init(hardwareMap);
        telemetry.addLine("Initialized all mechanism.");
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.init();
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyMotor");
        keeperServo = hardwareMap.get(Servo.class, "keeperServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        //liftServo = null;//hardwareMap.get(Servo.class, "liftServo");
        flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hoodServo.setPosition(0.0);
        //liftServo.setPosition(0.7);
    }

    public void start() {
        ringMotor.resetTimer();
    }

    @Override
    public void loop() {

        flyMotor.setVelocityPIDFCoefficients(80, 0, 10, 0); // Tune this for flywheel PID

        // Get mecanum drive inputs
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.drive(-forward, strafe, -rotate);

        aprTag.update();
        AprilTagDetection id20 = aprTag.getTagBySpecificId(tagNum);

        if (gamepad1.left_trigger > 0.3) {
            ringMotor.update(id20);
            if (id20 != null) {
                dx = id20.ftcPose.x;
                dy = id20.ftcPose.y;
                dz = id20.ftcPose.z;
                tagDistanceM = Math.sqrt(dx * dx + dy * dy + dz * dz);
                hoodCalc(tagDistanceM);
            }
        } else {
            ringMotor.stop();
        }



        // ---------------------- Drive inputs ---------------------------------------

        if (gamepad1.right_bumper) {
//            if (tagDistanceM < 170){
//                targetRPM = 3290;
//            } else if (tagDistanceM < 220){
//                targetRPM = 3590;
//            } else {
//                targetRPM = 3700;
//            }
            targetRPM=6000.0;
        } else {
            targetRPM = 0.0;
        }
        maxRPM = 6000.0;
        ticksPerSecond = (targetRPM * TICKS_PER_REV_FLY) / 60.0;
        maxTPS = (maxRPM * TICKS_PER_REV_FLY) / 60.0;
        flyMotor.setVelocity(ticksPerSecond);

        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
            //} else if (gamepad1.y) {
            //    intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        xNow = gamepad1.x;

        if (xNow && !xLast) {  // button JUST pressed
            isUp = !isUp;

            /*if (isUp) {
                liftServo.setPosition(0.0);
            } else {
                liftServo.setPosition(0.7);
            }*/
        }

        xLast = xNow;

        if (gamepad1.b) keeperServo.setPosition(FLICKER_PRESSED_POS);
        else keeperServo.setPosition(FLICKER_DEFAULT_POS);




        if (gamepad2.bWasPressed()) {stepIndex = (stepIndex + 1) % step.length;}
        if (gamepad2.dpadLeftWasPressed()) {ringMotor.setkP(ringMotor.getkP() - step[stepIndex]);}
        if (gamepad2.dpadRightWasPressed()) {ringMotor.setkP(ringMotor.getkP() + step[stepIndex]);}
        if (gamepad2.dpadUpWasPressed()) {ringMotor.setkD(ringMotor.getkD() - step[stepIndex]);}
        if (gamepad2.dpadDownWasPressed()) {ringMotor.setkD(ringMotor.getkD() + step[stepIndex]);}

        if (id20 != null) {
            telemetry.addLine("Tag detected.");
        } else {
            telemetry.addLine("No tag detected.");
        }

        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", ringMotor.getkP());
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", ringMotor.getkD());
        telemetry.addData("Step Size", "%.4f (B)", step[stepIndex]);
        telemetry.update();
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

        hoodServo.setPosition(SERVO_POS_INCREMENT * bestIndex);
        telemetry.addData("Hood index: %d", bestIndex);
    }
}
