package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 5.25; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -7; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        driver.setOffsets(PARAMS.parYTicks, PARAMS.perpXTicks, DistanceUnit.INCH);
        //double mmPerTick = inPerTick * 25.4;
        //driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
       // driver.setOffsets(133.35, 177.8 , DistanceUnit.MM);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        Pose2d p = null;
        try{
            txWorldPinpoint = pose.times(txPinpointRobot.inverse());
            System.out.print("SETPOSE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:");
        } catch (Exception e) {
            System.out.print("SETPOSE:"+e.getMessage());
            throw new RuntimeException(e);
        }
     }

    @Override
    public Pose2d getPose() {
        Pose2d p = null;
        try{
            p = txWorldPinpoint.times(txPinpointRobot);
            System.out.print("GETPOSE:!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        } catch (Exception e) {
            System.out.print("GETPOSE:"+e.getMessage());
            throw new RuntimeException(e);
        }
        return p;
    }

    public Pose2d getPoseInInch() {
        Pose2d p = null;
        try{
            driver.getPosX(DistanceUnit.INCH);
            System.out.print("GETPOSE:!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        } catch (Exception e) {
            System.out.print("GETPOSE:"+e.getMessage());
            throw new RuntimeException(e);
        }
        return p;
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
         if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
             double heading = driver.getHeading(UnnormalizedAngleUnit.RADIANS);
             txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), heading);
             Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
             Vector2d robotVelocity = Rotation2d.fromDouble(-heading).times(worldVelocity);
             return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
