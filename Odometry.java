package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.Math;

public class Odometry {
    GoBildaPinpointDriver odo;
    Pose2D pos;
    private IMU imu  = null;
    public Odometry(HardwareMap hardwareMap){
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(0, 0, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        odo.resetPosAndIMU();

    }
    public void loop(Gamepad gamepad, Telemetry telemetry, double targetBearing) {
        pos = odo.getPosition();
        odo.update();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double angle = orientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("X offset: ", pos.getX(DistanceUnit.INCH));
        telemetry.addData("Y offset: ", pos.getY(DistanceUnit.INCH));
        telemetry.addData("Gyro Heading: ", angle);
    }
    public double cur_X_pos(){
        pos=odo.getPosition();
        odo.update();
        return pos.getX(DistanceUnit.INCH);
    }
    public double cur_Y_pos(){
        pos=odo.getPosition();
        odo.update();
        return pos.getY(DistanceUnit.INCH);
    }

}
