package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import java.util.Queue;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;
import java.lang.Math;

@TeleOp(name="Joystick_Explosion", group="Titans TeleOps")

public class Joystick_Explosion extends OpMode {
    private DcMotorEx rightFront = null;
    private DcMotorEx leftFront = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx yeeterA = null;
    private DcMotorEx yeeterB = null;
    private double yeeterPower = 0.3;
    private DcMotorEx intakeA = null;
    //private CRServo intakeB =null;
    private CRServo feederA = null;
    private CRServo feederB = null;
    private CRServo feederC = null;
    private double intakePower = 0;
    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;
    private double motorFraction = 0.8;
    private int debounce = 0;
    private double integralSum = 0;
    private double Kp = 0.1;
    private double Ki = 0;
    private double Kd = 0;
    private double yeeterReference = 800;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private boolean yeeterActive = false;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //print that status = initialized
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        yeeterA = hardwareMap.get(DcMotorEx.class, "yeeterA");
        yeeterB = hardwareMap.get(DcMotorEx.class, "yeeterB");
        intakeA = hardwareMap.get(DcMotorEx.class, "intakeA");
        feederA = hardwareMap.get(CRServo.class, "feederA");
        feederB = hardwareMap.get(CRServo.class, "feederB");
        feederC = hardwareMap.get(CRServo.class, "feederC");
       // intakeB = hardwareMap.get(CRServo.class, "intakeB");
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //called every few milliseconds
        double leftFrontPower = (gamepad1.left_stick_y - gamepad1.right_stick_x ) - gamepad1.left_stick_x;
        double rightFrontPower = (gamepad1.left_stick_y + gamepad1.right_stick_x )  + gamepad1.left_stick_x;
        double leftBackPower = (gamepad1.left_stick_y + gamepad1.right_stick_x ) - gamepad1.left_stick_x;
        double rightBackPower = (gamepad1.left_stick_y - gamepad1.right_stick_x ) + gamepad1.left_stick_x;
        double motorSpeed = 1.0;
        if (gamepad1.right_trigger > 0.8){
            motorSpeed = 0.5;
        }
        if (gamepad1.right_bumper){
            motorFraction = 0.5;
        }
        leftFront.setPower(leftFrontPower * motorSpeed * motorFraction);
        rightFront.setPower(rightFrontPower * motorSpeed * motorFraction);
        leftBack.setPower(leftBackPower * motorSpeed * motorFraction);
        rightBack.setPower(rightBackPower * motorSpeed * motorFraction);


        if(gamepad2.dpad_up && debounce == 0 ) {
            yeeterReference += 10;

        }
        else if (gamepad2.dpad_down && debounce == 0) {
            yeeterReference -= 10;
        }
        double yeeterPowerA = PIDControl(yeeterReference, yeeterA.getVelocity());
        double yeeterPowerB = PIDControl(-yeeterReference, yeeterB.getVelocity());

        debounce = (debounce + 1) % 20;

        if(gamepad2.x || gamepad2.b) {
            feederA.setPower(1);
            feederB.setPower(1);
            feederC.setPower(1);
        }
        else {
            feederA.setPower(0);
            feederB.setPower(0);
            feederC.setPower(0);
        }
        if(gamepad2.y && debounce == 0){
            yeeterActive = !yeeterActive;
        }

        if(gamepad2.y || gamepad2.b) {
            yeeterA.setPower(0.325);
            yeeterB.setPower(-0.325);
        }
        else {
            yeeterA.setPower(0);
            yeeterB.setPower(0);
        }
        if(gamepad2.a || gamepad2.b) {
            intakeA.setPower(-1);
            feederA.setPower(1);
            feederB.setPower(1);
        }
        else {
            intakeA.setPower(0);
            feederA.setPower(0);
            feederB.setPower(0);
        }


        /* if(gamepad1.a) {
            yeeterA.setPower(0.3);
            yeeterB.setPower(-0.3);
            intakePower = 1;
            feederA.setPower(1);
            feederB.setPower(1);
            feederC.setPower(1);
        }
        else {
            yeeterA.setPower(0);
            yeeterB.setPower(0);
            intakePower = 0;
            feederA.setPower(0);
            feederB.setPower(0);
            feederC.setPower(0);
        }
        /*
        */


        /*if(gamepad1.x) {
            intakePower = 1;

        } else {
            intakePower = 0;
        }*/
        intakeA.setPower(-intakePower);

        //intakeB.setPower(intakePower);

        /* if(gamepad1.y) {
            feederA.setPower(1);
            feederB.setPower(1);
            feederC.setPower(1);
        } else {
            feederA.setPower(0);
            feederB.setPower(0);
            feederC.setPower(0);
        }*/
        dashboardTelemetry.addData("yeeterA velocity", yeeterA.getVelocity());
        dashboardTelemetry.addData("yeeterB velocity", -yeeterB.getVelocity());
        dashboardTelemetry.addData("leftBackPower", leftBackPower);
        dashboardTelemetry.addData("leftFrontPower", leftFrontPower);
        dashboardTelemetry.addData("rightFrontPower", rightFrontPower);
        dashboardTelemetry.addData("yeeterPowerA", yeeterPowerA);
        dashboardTelemetry.addData("yeeterPowerB", yeeterPowerB);
        dashboardTelemetry.addData("intakePower", intakePower);
        dashboardTelemetry.addData("debounce", debounce);

        dashboardTelemetry.update();





    }



    @Override
    public void stop() {
        stopMotors();
    }


    public void stopMotors() {
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

}

