/*
package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.lang.Math;

@Autonomous(name="Ministry_Medium_Machine", group="Robot")
public class Ministry_Medium_Machine extends LinearOpMode{
    GoBildaPinpointDriver odo;
    Pose2D pos;
    @Override
    public void runOpMode() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(0, 0, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
        waitForStart();
        odo.update();
    }


    public void update() {
        odo.update();

    }
    public void update_Tar(double at, double lt, double tx, double ty, double tt, double ts) {
        tar_pos_X = tx;
        tar_pos_Y  = ty;
    }
    public double dist_tar(){
        odo.update();
        pos = odo.getPosition();
        double distance = Math.sqrt(Math.pow((tar_pos_X - pos.getX(DistanceUnit.MM)), 2) + (Math.pow((tar_pos_Y - pos.getY(DistanceUnit.MM)), 2)));
        return(distance);
    }
    public void res_T(){
        srt_T = (double) System.currentTimeMillis()/1000.0;
        cur_T = (double) System.currentTimeMillis()/1000.0;
    }
    public double gt_T(){
        cur_T = (double) System.currentTimeMillis()/1000.0;
        return (cur_T- srt_T);
    }
    public void update_Timer(double Time){
        res_T();
        while(gt_T()<= Time){
            update();
            telemetry.update();
        }

    }
    public void update_X(double x){
        tar_pos_X = x;
    }
    public void update_Y(double y){
        tar_pos_Y = y;
    }

}

*/