package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive4Pinpoint;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive4Pinpoint.class)) {
            MecanumDrive4Pinpoint drive = new MecanumDrive4Pinpoint(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();


/*   FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry =
                    new MultipleTelemetry(telemetry, dashboard.getTelemetry());*/
               //Pose2d current =new Pose2d(0, 0, 0);
                /*Actions.runBlocking(
                        drive.actionBuilder(current)
                                .lineToX( 30  )
                                .build()
               );*/
            Pose2d start = drive.localizer.getPose();
            telemetry.addData("x", start.position.x);
            telemetry.addData("y", start.position.y);
            telemetry.addData("heading", start.heading.toDouble());
            telemetry.update();
           /* Actions.runBlocking(
                    drive.actionBuilder(start)
                            .splineToLinearHeading(
                                    new Pose2d(20, 20, 0),
                                    0.0
                            )
                            .build()
            );*/
           /*Actions.runBlocking(
                    drive.actionBuilder(start)
                            .strafeTo(new Vector2d(20, 20))
                            .build()
            );*/
            Actions.runBlocking(
                    drive.actionBuilder(start)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                           // .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
            /*  Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(30)
                            //.lineToX(0)
                            .build());*/
            /*Actions.runBlocking(
                    drive.actionBuilder(start)
                            .turnTo(Math.PI / 2)
                            .build()
            );

           Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToY(30)
                            .build()
            );*/
                while (opModeIsActive()) {
                    drive.updatePoseEstimate();
                    Pose2d pose = drive.localizer.getPose();
                    telemetry.addData(    "x", pose.position.x);
                    telemetry.addData("y", pose.position.y);
                    telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                    telemetry.update();
                    /*drive.leftFront.setPower(-0.3);
                    drive.leftBack.setPower(-0.3);
                    drive.rightFront.setPower(0.3);
                    drive.rightBack.setPower(0.3);
                   if (gamepad1.a) drive.leftFront.setPower(0.3);
                    else drive.leftFront.setPower(0);

                    if (gamepad1.b) drive.leftBack.setPower(0.3);
                    else drive.leftBack.setPower(0);

                    if (gamepad1.x) drive.rightFront.setPower(0.3);
                    else drive.rightFront.setPower(0);

                    if (gamepad1.y) drive. rightBack.setPower(0.3);
                    else drive.rightBack.setPower(0);*/
                }
        }
        else if(TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        } else {
            throw new RuntimeException();
        }
    }
}
