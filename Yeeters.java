package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Yeeters {
    private static final double kP = 0.005;
    private static final double kI = 0.00001;
    private static final double kD = 0.000001;

    private static final double alpha = 800;
    private static final double beta = 1.7;
    private static final double yeeterDefaultVelocity = 900;

    private DcMotorEx yeeterLeft = null;
    private DcMotorEx yeeterRight = null;
    private boolean yeeterActive = false;
    private double yeeterTargetVelocity = yeeterDefaultVelocity;

    private PID yeeterLeftPID = new PID(kP, kI, kD);
    private PID yeeterRightPID = new PID(kP, kI, kD);

    private int debounce = 0;

    public  Yeeters(HardwareMap hardwareMap) {
        // Get yeeters from hardware map
        yeeterLeft = hardwareMap.get(DcMotorEx.class, "yeeterLeft");
        yeeterRight = hardwareMap.get(DcMotorEx.class, "yeeterRight");

        // Reverse direction of right Yeeter
        yeeterRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry, double distance) {
        // Toggle yeeter active based on Y button and previous button state
        if (gamepad.yWasPressed()){
            yeeterActive = !yeeterActive;
        }

        if (distance == 0){
            yeeterTargetVelocity = yeeterDefaultVelocity;
        } else {
            // Automatic Target Velocity using Linear Regression
            yeeterTargetVelocity = computeYeeterVelocity(distance);
        }

        // Adjust Target Velocity
        /*if (gamepad.dpad_up && debounce == 0) {
            yeeterTargetVelocity += 10;
        } else if (gamepad.dpad_down && debounce == 0) {
            yeeterTargetVelocity -= 10;
        }*/

        debounce = (debounce + 1) % 60;

        // Set yeeter powers if active or B button is pressed using proportional
        double yeeterLeftPower = 0;
        double yeeterRightPower = 0;
        if (yeeterActive || gamepad.b ) {
          yeeterLeftPower = yeeterLeftPID.update(yeeterTargetVelocity, yeeterLeft.getVelocity());
          yeeterRightPower = yeeterRightPID.update(yeeterTargetVelocity, yeeterRight.getVelocity());
        }

        yeeterLeft.setPower(yeeterLeftPower);
        yeeterRight.setPower(yeeterRightPower);

        //add T
        telemetry.addData("distance", distance);
        telemetry.addData("yeeterTargetVelocity", yeeterTargetVelocity);
        telemetry.addData("yeeterRightVelocity", yeeterRight.getVelocity());
        telemetry.addData("yeeterLeftVelocity", yeeterLeft.getVelocity());
        telemetry.addData("yeeterRightPower", yeeterRightPower);
        telemetry.addData("yeeterLeftPower", yeeterLeftPower);
    }
    public void stop() {
        //stop all motors
        yeeterLeft.setPower(0);
        yeeterRight.setPower(0);
    }

    public void toggleYeeters() {
        yeeterActive = !yeeterActive;
    }

    public double computeYeeterVelocity(double distance) {
        return alpha + beta*distance;
    }
}