package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Yeeters {
    private static final double kP = 200;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 12.5;

    private static final double EPSILON = 0.03;
    private static final double alpha = 700.0; // 715
    private static final double beta = 1.65; // 160
    private static final double yeeterNearVelocity = 805;
    private static final double yeeterFarVelocity = 960;
    private static final double yeeterSuperFarVelocity = 1000;
    private DcMotorEx yeeterLeft = null;
    private DcMotorEx yeeterRight = null;
    private boolean yeeterActive = false;
    private double yeeterTargetVelocity = yeeterNearVelocity;

    private boolean yeeterManual = false;

    public  Yeeters(HardwareMap hardwareMap) {
        // Set PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);

        // Setup left Yeeter
        yeeterLeft = hardwareMap.get(DcMotorEx.class, "yeeterLeft");
        yeeterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        yeeterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Setup right Yeeter
        yeeterRight = hardwareMap.get(DcMotorEx.class, "yeeterRight");
        yeeterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        yeeterRight.setDirection(DcMotorEx.Direction.REVERSE);
        yeeterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry, double distance) {
        // Toggle yeeter active based on Y button and previous button state
        if (gamepad.yWasPressed()){
            yeeterActive = !yeeterActive;
        }

        if (gamepad.rightBumperWasPressed()) {
            yeeterManual = !yeeterManual;
        }

        if (gamepad.left_bumper) {
            yeeterTargetVelocity = yeeterFarVelocity;
        } else if (gamepad.left_trigger > 0.8) {
            yeeterTargetVelocity = yeeterSuperFarVelocity;
        } else if (yeeterManual) {
            if (gamepad.dpadUpWasPressed()) {
                yeeterTargetVelocity += 5;
            } else if(gamepad.dpadDownWasPressed()) {
                yeeterTargetVelocity -= 5;
            }
        } else {
            if (distance == 0) {
                yeeterTargetVelocity = yeeterNearVelocity;
            } else {
                // Automatic Target Velocity using Linear Regression
                yeeterTargetVelocity = computeYeeterVelocity(distance);
            }
        }

        // Set yeeter velocities
        if (yeeterActive) {                  //Fix Swerve
            yeeterLeft.setVelocity(yeeterTargetVelocity);
            yeeterRight.setVelocity(yeeterTargetVelocity);
        } else {
            yeeterLeft.setVelocity(0);
            yeeterRight.setVelocity(0);
        }

        //add T
        telemetry.addData("distance", distance);
        telemetry.addData("yeeterTargetVelocity", yeeterTargetVelocity);
        telemetry.addData("yeeterRightVelocity", yeeterRight.getVelocity());
        telemetry.addData("yeeterLeftVelocity", yeeterLeft.getVelocity());
    }
    public void stop() {
        //stop all motors
        yeeterLeft.setPower(0);
        yeeterRight.setPower(0);
    }

    public void toggleYeeters() {
        yeeterActive = !yeeterActive;
        if (!yeeterActive){
            stop();
        }
    }

    public double computeYeeterVelocity(double distance) {
        return alpha + beta * distance;
    }

    public boolean isReady() {
        return (Math.abs (yeeterLeft.getVelocity()- yeeterTargetVelocity) <EPSILON * yeeterTargetVelocity) &&
                (Math.abs (yeeterRight.getVelocity()- yeeterTargetVelocity) <EPSILON * yeeterTargetVelocity);
    }
}