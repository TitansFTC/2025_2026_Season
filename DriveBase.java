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
    public static final double[] TARGET_REACHED    = {0, 0, 0, 0, 0};

    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    private double tar_pos_x;
    private double tar_pos_y;
    private double tar_angle;

    public DriveBase(HardwareMap hardwareMap) {
        // Get motors from hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Reverse direction of motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry, double targetBearing, double cur_pos_x, double cur_pos_y, double cur_angle) {
        // Compute left motor powers
        double leftFrontPower = (gamepad.left_stick_y - gamepad.right_stick_x) - gamepad.left_stick_x;
        double leftBackPower = (gamepad.left_stick_y + gamepad.right_stick_x) - gamepad.left_stick_x;

        // Compute right motor powers
        double rightFrontPower = (gamepad.left_stick_y + gamepad.right_stick_x) + gamepad.left_stick_x;
        double rightBackPower = (gamepad.left_stick_y - gamepad.right_stick_x) + gamepad.left_stick_x;

        // Reduce speed based on right trigger
        double powerFraction = FAST_POWER_FRACTION;
        if (gamepad.right_trigger > 0.8) {
            powerFraction = SLOW_POWER_FRACTION;
        }

        // Hold position
        if (gamepad.leftTriggerWasPressed()){
            tar_pos_x = cur_pos_x;
            tar_pos_y = cur_pos_y;
            tar_angle = cur_angle;
        }
        if (gamepad.left_trigger > 0.8) {
            odoMove(tar_pos_x, tar_pos_y, tar_angle, cur_pos_x, cur_pos_y, cur_angle);
            return; // Skip rest if we are holding position
        }

        // Auto aim
        if (gamepad.b && Math.abs(targetBearing) > 0.5 && Math.abs(targetBearing) < 180) {
                powerFraction = Math.max(SLOW_POWER_FRACTION*0.50, Math.abs(targetBearing) / 100);

                if (targetBearing > 0) {
                    leftFrontPower = FAST_POWER_FRACTION;
                    leftBackPower = -FAST_POWER_FRACTION;
                    rightFrontPower = -FAST_POWER_FRACTION;
                    rightBackPower = FAST_POWER_FRACTION;
                } else {
                    leftFrontPower = -FAST_POWER_FRACTION;
                    leftBackPower = FAST_POWER_FRACTION;
                    rightFrontPower = FAST_POWER_FRACTION;
                    rightBackPower = -FAST_POWER_FRACTION;
                }
        }

        // Set left and right motor powers
        leftFront.setPower(leftFrontPower * powerFraction);
        leftBack.setPower(leftBackPower * powerFraction);
        rightFront.setPower(rightFrontPower * powerFraction);
        rightBack.setPower(rightBackPower * powerFraction);

        // Add Telemetry
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

    public boolean odoMove(double tar_pos_x, double tar_pos_y, double tar_angle, double cur_pos_x, double cur_pos_y, double cur_angle) {
        double[] powers = odoCompute(tar_pos_x, tar_pos_y, tar_angle, cur_pos_x, cur_pos_y, cur_angle);

        leftFront.setPower(powers[0] * powers[4]);
        leftBack.setPower(powers[1] * powers[4]);
        rightFront.setPower(powers[2] * powers[4]);
        rightBack.setPower(powers[3] * powers[4]);

        return powers == TARGET_REACHED;
    }

    public double[] odoCompute(double tar_pos_x, double tar_pos_y, double tar_angle, double cur_pos_x, double cur_pos_y, double cur_angle){
        double rel_tar_x = tar_pos_x - cur_pos_x;
        double rel_tar_y = tar_pos_y - cur_pos_y;

        // Skip if we have reached target
        if ((rel_tar_x == 0) && (rel_tar_y == 0) && (tar_angle == cur_angle)) {
            return TARGET_REACHED;
        }

        double beta = 90;
        if (rel_tar_y < 0) {
            beta = -90;
        }
        if (rel_tar_x != 0) {
            beta = Math.toDegrees(Math.atan(rel_tar_y / rel_tar_x));
        }
        if (rel_tar_x < 0) {
            beta = beta - 180;
        }

        double pos_angle = 90 + cur_angle - beta;
        double rel_pos_x = Math.sin(Math.toRadians(pos_angle));
        double rel_pos_y = Math.cos(Math.toRadians(pos_angle));
        double rel_angle = 0;

        if (Math.abs(cur_angle - tar_angle) >= 30) {
            if (tar_angle > cur_angle) {
                rel_angle =  0.5;
            } else {
                rel_angle = -0.5;
            }
        } else {
            rel_angle = ((tar_angle - cur_angle) / 30) * 0.5;
        }

        double powerFraction = FAST_POWER_FRACTION;
        double distanceAway = Math.sqrt(rel_pos_x*rel_pos_x + rel_pos_y*rel_pos_y);
        if (distanceAway < 6.0) {
            powerFraction = Math.max(SLOW_POWER_FRACTION, distanceAway / 10);
        }

        double lfp = ((rel_pos_y + -rel_pos_x) - rel_angle);
        double lbp = ((rel_pos_y +  rel_pos_x) - rel_angle);
        double rfp = ((rel_pos_y +  rel_pos_x) + rel_angle);
        double rbp = ((rel_pos_y + -rel_pos_x) + rel_angle);

        if ((Math.abs(lfp) >= 1) || (Math.abs(rfp) >= 1) || (Math.abs(lbp) >= 1) || (Math.abs(rbp) >= 1)) {
            double highestPower = Math.max(Math.max(Math.abs(lfp), Math.abs(rfp)), Math.max(Math.abs(rbp), Math.abs(lbp)));
            lfp = lfp / highestPower;
            lbp = lbp / highestPower;
            rfp = rfp / highestPower;
            rbp = rbp / highestPower;
        }

        double[] powers = {lfp, lbp, rfp, rbp, powerFraction};
        return powers;
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

    public void moveBackward(double power, int duration) {
        int initial = rightBack.getCurrentPosition();
        int difference = Math.abs(initial - rightBack.getCurrentPosition());
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
}