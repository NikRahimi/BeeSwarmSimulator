package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretTest {
    private DcMotorEx ringMotor;
    private double kP = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;
    private double goalX = 0.0;
    private double lastError = 0.0;
    private double angleTol = 3;
    private final double MAX_POW = 0.8;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    void init(HardwareMap hwMap) {
        ringMotor = hwMap.get(DcMotorEx.class, "ringMotor");
        ringMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ringMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // Mutator
    public void setkP(double newkP) {kP = newkP;}
    public void setkD(double newkD) {kD = newkD;}

    // Accessor
    public double getkP() {return kP;}
    public double getkD() {return kD;}

    public void resetTimer() {timer.reset();}

    public void stop() {
        ringMotor.setPower(0);
        lastError = 0;
    }

    public void update(AprilTagDetection curID) {
        double deltaTime = timer.seconds();
        timer.reset();
        if (curID == null) {
            ringMotor.setPower(0);
            lastError = 0;
            return;
        }

        // Start pD controller
        double error = goalX - curID.ftcPose.bearing;
        double pTerm = error * kP;
        double dTerm = 0;

        if (deltaTime > 0) {dTerm = ((error - lastError) / deltaTime) * kD;}

        if (Math.abs(error) < angleTol) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POW, MAX_POW);
        }

        // ADD SOFT LIMITITSISISISIIISISISIISSISIISISISISISISISISIISISISIS

        ringMotor.setPower(- power);
        lastError = error;
    }
}
