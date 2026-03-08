package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveBase {
    public static final double FAST_POWER_FRACTION = 1.0;
    public static final double SLOW_POWER_FRACTION = 0.2;
    public static final double HALF_POWER_FRACTION = 0.5;

    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;
    private double tar_X;
    private double tar_Y;
    private double tar_H;

    public DriveBase(HardwareMap hardwareMap) {
        //Get motors from HM
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        //Reverse direction of motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    public void loop(Gamepad gamepad, Telemetry telemetry, double targetBearing, double cur_Pos_X, double cur_Pos_Y, double cur_H) {
        //Compute left motor powers
        double leftFrontPower = (gamepad.left_stick_y - gamepad.right_stick_x) - gamepad.left_stick_x;
        double leftBackPower = (gamepad.left_stick_y + gamepad.right_stick_x) - gamepad.left_stick_x;

        //Compute right motor powers
        double rightFrontPower = (gamepad.left_stick_y + gamepad.right_stick_x) + gamepad.left_stick_x;
        double rightBackPower = (gamepad.left_stick_y - gamepad.right_stick_x) + gamepad.left_stick_x;

        //Reduce speed based on right trigger
        double powerFraction = FAST_POWER_FRACTION;
        if (gamepad.right_trigger > 0.8) {
            powerFraction = SLOW_POWER_FRACTION;
        }
        //Hold position
        if (gamepad.leftTriggerWasPressed()){
            tar_X = cur_Pos_X;
            tar_Y = cur_Pos_Y;
            tar_H = cur_H;
        }
        if (gamepad.left_trigger>.8){
            holdPosition(tar_X, tar_Y, tar_H, cur_Pos_X, cur_Pos_Y, cur_H);
        }

        // Auto aim
        if (gamepad.b && Math.abs(targetBearing) > 0.5 && Math.abs(targetBearing) < 180) {
                powerFraction = Math.max(SLOW_POWER_FRACTION*0.50, Math.abs(targetBearing) / 100);

                if (targetBearing>0) {
                    leftFrontPower = FAST_POWER_FRACTION;
                    leftBackPower = -FAST_POWER_FRACTION;
                    rightFrontPower = -FAST_POWER_FRACTION;
                    rightBackPower = FAST_POWER_FRACTION;

                }else {
                    leftFrontPower = -FAST_POWER_FRACTION;
                    leftBackPower = FAST_POWER_FRACTION;
                    rightFrontPower = FAST_POWER_FRACTION;
                    rightBackPower = -FAST_POWER_FRACTION;

                }

        }

        //Set left and right motor powers
        leftFront.setPower(leftFrontPower * powerFraction);
        leftBack.setPower(leftBackPower * powerFraction);
        rightFront.setPower(rightFrontPower * powerFraction);
        rightBack.setPower(rightBackPower * powerFraction);

        //Add T
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);
        telemetry.addData("powerFraction", powerFraction);
        telemetry.addData("rightBack" , rightBack.getCurrentPosition());
        telemetry.addData("targetBearing", targetBearing);
    }

    public void stop() {
        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void moveForward(double power, int duration) {
        int initial = rightBack.getCurrentPosition();
        int difference = Math.abs(initial - rightBack.getCurrentPosition());
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        while (difference < duration) {
            difference = Math.abs(initial - rightBack.getCurrentPosition());
        }
    }
    public void odoMove(double tar_pos_X, double tar_pos_Y, double tar_T, double cur_Pos_X, double cur_Pos_Y, double cur_Heading){

       double rel_tar_X = tar_pos_X - cur_Pos_X;
       double rel_tar_Y = tar_pos_Y - cur_Pos_Y;
        if ((rel_tar_X != 0) || (rel_tar_Y != 0) || (tar_T != cur_Heading)) {
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
            double C = 0; // What is C?
            double A = 90 + C - beta;
            double rel_X = Math.sin(Math.toRadians(A));
            double rel_Y = Math.cos(Math.toRadians(A));
            double rel_T = 0;
            if (Math.abs(cur_Heading - tar_T) >= 30) {
                if (tar_T > cur_Heading){
                    rel_T = .5;
                }
                else {
                    rel_T = -.5;
                }
            }
            else {
                rel_T = ((tar_T - cur_Heading )/30) * .5;
            }

            double slow = 1;
            if (Math.sqrt(Math.pow((tar_pos_X - cur_Pos_X), 2) + (Math.pow((tar_pos_Y - cur_Pos_Y), 2))) < 7.87){
                slow = Math.sqrt(Math.pow((tar_pos_X - cur_Pos_X), 2) + (Math.pow((tar_pos_Y - cur_Pos_Y), 2)))/5080;
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
            leftFront.setPower(lfp);
            rightFront.setPower(rfp);
            leftBack.setPower(lbp);
            rightBack.setPower(rbp);
        }
    }

    public void moveBackward(double power, int duration) {
        int initial = rightBack.getCurrentPosition();
        int difference = Math.abs(initial - rightBack.getCurrentPosition());
        dashboardTelemetry.addData("Difference: ", difference);
        dashboardTelemetry.update();
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
        while (difference < duration) {
            difference = Math.abs(initial - rightBack.getCurrentPosition());
        }
    }

    public void rotateRight(double power, int duration) {
        int initial = rightBack.getCurrentPosition();
        int difference = Math.abs(initial - rightBack.getCurrentPosition());
        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(-power);
        while (difference < duration) {
            difference = Math.abs(initial - rightBack.getCurrentPosition());
        }
    }

    public void rotateLeft(double power, int duration) {
        int initial = rightBack.getCurrentPosition();
        int difference = Math.abs(initial - rightBack.getCurrentPosition());
        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(power);
        while (difference < duration) {
            difference = Math.abs(initial - rightBack.getCurrentPosition());
        }
    }

    public void strafeRight(double power, int duration) {
        int initial = rightBack.getCurrentPosition();
        int difference = Math.abs(initial - rightBack.getCurrentPosition());
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        while (difference < duration) {
            difference = Math.abs(initial - rightBack.getCurrentPosition());
        }
    }

    public void strafeLeft(double power, int duration) {
        int initial = rightBack.getCurrentPosition();
        int difference = Math.abs(initial - rightBack.getCurrentPosition());
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
        while (difference < duration) {
            difference = Math.abs(initial - rightBack.getCurrentPosition());
        }
    }

    public void holdPosition(double tar_Pos_X, double tar_Pos_Y, double tar_H, double cur_Pos_X, double cur_Pos_Y, double cur_H) {
        odoMove(tar_Pos_X, tar_Pos_Y, tar_H, cur_Pos_X, cur_Pos_Y, cur_H);
    }

}