package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveBase {
    private static final double FAST_POWER_FRACTION = 0.8;
    private static final double SLOW_POWER_FRACTION = 0.4;

    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

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
    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        //Compute left motor powers
        double leftFrontPower = (gamepad.left_stick_y - gamepad.right_stick_x ) - gamepad.left_stick_x;
        double leftBackPower = (gamepad.left_stick_y + gamepad.right_stick_x ) - gamepad.left_stick_x;

        //Compute right motor powers
        double rightFrontPower = (gamepad.left_stick_y + gamepad.right_stick_x )  + gamepad.left_stick_x;
        double rightBackPower = (gamepad.left_stick_y - gamepad.right_stick_x ) + gamepad.left_stick_x;

        //Reduce speed based on right trigger
        double powerFraction = FAST_POWER_FRACTION;
        if (gamepad.right_trigger > 0.8) {
            powerFraction = SLOW_POWER_FRACTION;
        }

        //Set left and right motor powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        //Add T
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);
        telemetry.addData("powerFraction", powerFraction);
    }

    public void stop() {
        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}