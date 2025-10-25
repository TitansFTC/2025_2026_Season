package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private DcMotor yeeterA = null;
    private DcMotor yeeterB = null;
    private double yeeterPower = 0;
    private DcMotor intakeA = null;
    //private CRServo intakeB =null;
    private CRServo feederA = null;
    private CRServo feederB = null;
    private CRServo feederC = null;
    private double intakePower = 0;
    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;
    private double motorDivider = 1;
    private int debounce = 0;
    private int yeeterAOldPosition = 0;
    private int yeeterBOldPosition = 0;
    private double oldTime = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //print that status = initialized
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        yeeterA = hardwareMap.get(DcMotor.class, "yeeterA");
        yeeterB = hardwareMap.get(DcMotor.class, "yeeterB");
        intakeA = hardwareMap.get(DcMotor.class, "intakeA");
        feederA = hardwareMap.get(CRServo.class, "feederA");
        feederB = hardwareMap.get(CRServo.class, "feederB");
        feederC = hardwareMap.get(CRServo.class, "feederC");
       // intakeB = hardwareMap.get(CRServo.class, "intakeB");
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        yeeterAOldPosition = yeeterA.getCurrentPosition();
        yeeterBOldPosition = yeeterB.getCurrentPosition();
        oldTime = System.currentTimeMillis()/1000.0;

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
        double leftFrontPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x ) + gamepad1.right_stick_x;
        double rightFrontPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x )  - gamepad1.right_stick_x;
        double leftBackPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x ) + gamepad1.right_stick_x;
        double rightBackPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x ) - gamepad1.right_stick_x;
        double motorSpeed = 1.0;
        if (gamepad1.right_trigger > 0.8){
            motorSpeed = 0.5;
        }
        if (gamepad1.right_bumper){
            motorDivider = 2;
        }
        leftFront.setPower(leftFrontPower * motorSpeed / motorDivider);
        rightFront.setPower(rightFrontPower * motorSpeed / motorDivider);
        leftBack.setPower(leftBackPower * motorSpeed / motorDivider);
        rightBack.setPower(rightBackPower * motorSpeed / motorDivider);

        if(gamepad1.dpad_up && debounce == 0 ) {
            yeeterPower= Math.min(1, yeeterPower+0.01);

        }
        else if (gamepad1.dpad_down && debounce == 0) {
            yeeterPower= Math.max(0, yeeterPower- 0.01);
        }

        debounce = (debounce + 1) % 100;


        if(gamepad1.a) {
            yeeterA.setPower(yeeterPower);
            yeeterB.setPower(-yeeterPower);
        }
        else {
            yeeterA.setPower(0);
            yeeterB.setPower(0);
        }


        if(gamepad1.x) {
            intakePower = 1;

        } else {
            intakePower = 0;
        }
        intakeA.setPower(-intakePower);

        //intakeB.setPower(intakePower);

        if(gamepad1.y) {
            feederA.setPower(1);
            feederB.setPower(1);
            feederC.setPower(1);
        } else {
            feederA.setPower(0);
            feederB.setPower(0);
            feederC.setPower(0);
        }

        int yeeterANewPosition = yeeterA.getCurrentPosition();
        int yeeterBNewPosition = yeeterB.getCurrentPosition();
        double newTime = System.currentTimeMillis() / 1000.0;
        double yeeterARate = (yeeterANewPosition - yeeterAOldPosition) / (newTime - oldTime);
        double yeeterBRate = Math.abs((yeeterBNewPosition - yeeterBOldPosition) / (newTime - oldTime));
        oldTime = newTime;
        yeeterAOldPosition = yeeterANewPosition;
        yeeterBOldPosition = yeeterBNewPosition;

        dashboardTelemetry.addData("yeeterA rate", yeeterARate);
        dashboardTelemetry.addData("yeeterB rate", yeeterBRate);
        dashboardTelemetry.addData("leftBackPower", leftBackPower);
        dashboardTelemetry.addData("leftFrontPower", leftFrontPower);
        dashboardTelemetry.addData("rightFrontPower", rightFrontPower);
        dashboardTelemetry.addData("yeeterPower", yeeterPower);
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
}

