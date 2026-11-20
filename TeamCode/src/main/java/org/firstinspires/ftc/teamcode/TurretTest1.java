package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Bare-bones turret diagnostic. Does NOT use Road Runner, tracking, or anything else.
 * Left stick Y spins the turret motor at up to 30% power.
 * Watch whether the motor spins and which way the ticks move.
 */
@TeleOp(name = "TurretTest", group = "Test")
public class TurretTest1 extends OpMode {

    private DcMotorEx turretMotor;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Ready. Push left stick up/down to spin turret motor.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = -gamepad1.left_stick_y * 0.3;
        turretMotor.setPower(power);

        telemetry.addData("Commanded power", "%.2f", power);
        telemetry.addData("Encoder ticks", turretMotor.getCurrentPosition());
        telemetry.addData("Velocity (ticks/s)", "%.0f", turretMotor.getVelocity());
        telemetry.addData("Motor current (A)", "%.2f", turretMotor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS));
        telemetry.update();
    }
}