package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;


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
    private DcMotor intakeA =null;
    //private CRServo intakeB =null;
    private double intakePower = 0;

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
       // intakeB = hardwareMap.get(CRServo.class, "intakeB");


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //moveForward(0.5, 500);
        //stopMotors(0);
    }

    @Override
    public void loop() {
        //called every few milliseconds
        double leftFrontPower = (-gamepad1.left_stick_y + -gamepad1.right_stick_x ) - gamepad1.left_stick_x;
        double leftBackPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x ) - gamepad1.left_stick_x;
        double rightFrontPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x )  + gamepad1.left_stick_x;
        double rightBackPower = (-gamepad1.left_stick_y + -gamepad1.right_stick_x ) + gamepad1.left_stick_x;
        double motorSpeed = 1.0;
        if (gamepad1.right_trigger > 0.8){
            motorSpeed = 0.5;
        }
        leftFront.setPower(leftFrontPower * motorSpeed);
        leftBack.setPower(leftBackPower * motorSpeed);
        rightFront.setPower(rightFrontPower * motorSpeed);
        rightBack.setPower(rightBackPower * motorSpeed);

        if(gamepad1.a) {
            yeeterPower = 1;
        }
        else if (gamepad1.b) {
            yeeterPower = -1;
        }
        else {
            yeeterPower = 0;
        }
        yeeterA.setPower(yeeterPower);
        yeeterB.setPower(-yeeterPower);

        if(gamepad1.x) {
            intakePower = 1;
        } else if (gamepad1.y) {
            intakePower = -1;
        } else {
            intakePower = 0;
        }
        intakeA.setPower(-intakePower);
        //intakeB.setPower(intakePower);


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

