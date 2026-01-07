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
public class Ministry_TEST extends LinearOpMode{

    private ElapsedTime     runtime = new ElapsedTime();
    static final double TICKS_PER_INCH = 2000.0 / 44.0;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private double START_HEADING;
    private double rs;

    GoBildaPinpointDriver odo;

    double tar_Posit_ARM;
    double cur_Posit_ARM;
    double rem_Dis_ARM;
    double prop_Cont_Power_ARM;
    double killa = 500;
    double prop_SPEED = .9;
    double str_Posit;
    double cur_Posit = 0;
    double tar_Pos_ARM_MAIN;
    double kill = 500;
    double tp;
    double cp;
    double gp;
    double pcp;
    double Target_Posit;
    double tar_pos_X;
    double tar_pos_Y;
    double A;
    double C;
    Pose2D pos;
    double rel_X;
    double rel_Y;
    double rel_tar_X;
    double rel_tar_Y;
    double cur_T;
    double srt_T;
    double tar_T;
    double rel_T;
    double tar_SPINNY;
    double z_OFFSET;


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
    public void drive_distance(double left_inches, double right_inches) {
        double goal_left = left_inches * TICKS_PER_INCH;
        double goal_right = right_inches * TICKS_PER_INCH;
        double rp;
        double lp;
        double rpt;
        double lpt;
        rp = rf.getCurrentPosition();
        lp = lb.getCurrentPosition();
        if (goal_right > 0) {
            rf.setPower(.7);
            rb.setPower(.7);
        }
        if (goal_right < 0) {
            rf.setPower(-.7);
            rb.setPower(-.7);
        }
        if (goal_left > 0){
            lf.setPower(.7);
            lb.setPower(.7);
        }
        if (goal_left < 0){
            lf.setPower(-.7);
            lb.setPower(-.7);
        }
        rpt = rp + goal_right;
        lpt = lp + goal_left;
        while ((goal_right > 0 && rp < rpt) || (goal_right < 0 && rp > rpt)
                || (goal_left > 0 && lp < lpt) || (goal_left < 0 && lp > lpt)) {
            sleep(50);
            rp = rf.getCurrentPosition();
            lp = lb.getCurrentPosition();
            if ((goal_right > 0 && rp > rpt) || (goal_right < 0 && rp < rpt)) {
                rf.setPower(0);
                rb.setPower(0);
            }
            if ((goal_left > 0 && lp >  lpt) || (goal_left < 0 && lp < lpt)) {
                lf.setPower(0);
                lb.setPower(0);
            }
        }
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
    public void linear_distance (double up_inches) {
        double up = abs(le2.getCurrentPosition());
        telemetry.addData("Lin: ", up);
        double goal_up = up_inches + up;
        telemetry.addData("Line: ", goal_up);
        if (up_inches > 0) {
            le.setPower(.8);
            le2.setPower(.8);
        }
        if (up_inches < 0) {
            le.setPower(-.8);
            le2.setPower(-.8);
        }
        while ((up_inches > 0 && up < goal_up) || (up_inches < 0 && up > goal_up)
        ) {
            sleep(50);
            up = abs(le2.getCurrentPosition());
            telemetry.addData("Linear:", up);
        }
        le.setPower(0);
        le2.setPower(0);
    }
    /*public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void turn_goal (double turn_goal, boolean turn_right){
        double turn_posit = translate(START_HEADING + turn_goal);
        double cur_posit = translate(getHeading());
        if (turn_right == false){
            //left
            while (cur_posit < turn_posit){
                lf.setPower(-.5);
                rf.setPower(.5);
                lb.setPower(-.5);
                rb.setPower(.5);
                cur_posit = translate(getHeading());
            }
            stop_drive();
            return;
        }else {
            //right
            while (cur_posit > turn_posit){
                lf.setPower(.5);
                rf.setPower(-.5);
                lb.setPower(.5);
                rb.setPower(-.5);
                cur_posit = translate(getHeading());
            }
            stop_drive();
            return;
        }


    public void update() {
        odo.update();
        spinny.setPower(tar_SPINNY);

        tar_Posit_ARM = tar_Pos_ARM_MAIN;
        cur_Posit_ARM = ar.getCurrentPosition();
        rem_Dis_ARM = tar_Posit_ARM - cur_Posit_ARM;
        str_Posit = cur_Posit;
        cur_Posit = ar.getCurrentPosition();
        double posit_Diff = cur_Posit - str_Posit;
        if (abs(rem_Dis_ARM) > killa){
            if (rem_Dis_ARM > 0){
                prop_Cont_Power_ARM = prop_SPEED;
            }
            if (rem_Dis_ARM < 0){
                prop_Cont_Power_ARM = -prop_SPEED;
            }
        } else {
            prop_Cont_Power_ARM = (rem_Dis_ARM/killa)*prop_SPEED + posit_Diff * .005;
        }
        ar.setPower(prop_Cont_Power_ARM);
        tp = Target_Posit;
        cp = le2.getCurrentPosition() - z_OFFSET;
        gp = tp - cp;
        if (abs(gp) > kill){
            if (gp > 0){
                pcp = -1;
            }
            if (gp < 0){
                pcp = 1;
            }
        } else {
            pcp = -gp/kill;
        }
        le.setPower(pcp);
        le2.setPower(pcp);
        pos = odo.getPosition();
        C = pos.getHeading(AngleUnit.DEGREES);
        rel_tar_X = tar_pos_X - pos.getX(DistanceUnit.MM);
        rel_tar_Y = tar_pos_Y - pos.getY(DistanceUnit.MM);
        if ((rel_tar_X != 0) || (rel_tar_Y != 0) || (tar_T != pos.getHeading(AngleUnit.DEGREES))) {
            double beta = 90;
            if (rel_tar_Y < 0) {
                beta = -90;
            }
            if (rel_tar_X != 0 ){
                beta = Math.toDegrees(Math.atan(rel_tar_Y/rel_tar_X));
            }
            if (rel_tar_X < 0){
                beta = beta - 180;
            }
            A = 90 + C - beta;
            rel_X = Math.sin(Math.toRadians(A));
            rel_Y = Math.cos(Math.toRadians(A));
            if (Math.abs(pos.getHeading(AngleUnit.DEGREES) - tar_T) >= 30) {
                if (tar_T > pos.getHeading(AngleUnit.DEGREES)){
                    rel_T = .5;
                }
                else {
                    rel_T = -.5;
                }
            }
            else {
                rel_T = ((tar_T - pos.getHeading(AngleUnit.DEGREES) )/30) * .5;
            }
            telemetry.update();
            telemetry.addData("rel_X: ", rel_X);
            telemetry.addData("rel_Y: ", rel_Y);
            telemetry.addData("Heading (pos): ", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("rel_tar_X: ", rel_tar_X);
            telemetry.addData("rel_tar_Y: ", rel_tar_Y);
            //telemetry.addData("Beta: ", beta);
            //telemetry.addData("A: ", A);
            //telemetry.addData("C: ", C);
            telemetry.addData("odoX: ", pos.getX(DistanceUnit.MM));
            telemetry.addData("odoY: ", pos.getY(DistanceUnit.MM));
            telemetry.addData("Arm: ", tar_Pos_ARM_MAIN);
            telemetry.addData("Arm_Pow; ", prop_Cont_Power_ARM);
            telemetry.update();
            double slow = 1;
            if (dist_tar() < 200){
                slow = dist_tar()/200;
            }
            double lfp = ((rel_Y + rel_X )  * slow - rel_T);
            double rfp = ((rel_Y + -rel_X )  * slow + rel_T);
            double lbp = ((rel_Y + -rel_X )  * slow - rel_T);
            double rbp = ((rel_Y + rel_X )  * slow + rel_T);
            if ((Math.abs(lfp) >= 1) || (Math.abs(rfp) >= 1) || (Math.abs(lbp) >= 1) || (Math.abs(rbp) >= 1)){
                double k = Math.max(Math.max(Math.abs(lfp), Math.abs(rfp)), Math.max(Math.abs(rbp), Math.abs(lbp)));
                lfp = lfp/k;
                rfp = rfp/k;
                rbp = rbp/k;
                lbp = lbp/k;
            }
            lf.setPower(lfp);
            rf.setPower(rfp);
            lb.setPower(lbp);
            rb.setPower(rbp);
        }


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