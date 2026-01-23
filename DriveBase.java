package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveBase {
    public static final double FAST_POWER_FRACTION = 1.0;
    public static final double SLOW_POWER_FRACTION = 0.2;

    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;

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

    public void loop(Gamepad gamepad, Telemetry telemetry) {
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
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(-power);
        while (difference < duration) {
            difference = Math.abs(initial - rightBack.getCurrentPosition());
        }
    }

    public void strafeLeft(double power, int duration) {
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
}