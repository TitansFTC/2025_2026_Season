package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Yeeters {
    private static final double kP = 0.005;

    private DcMotorEx yeeterLeft = null;
    private DcMotorEx yeeterRight = null;
    private boolean yeeterActive = false;
    private double yeeterVelocityTarget = 800;

    public  Yeeters(HardwareMap hardwareMap) {
        // Get yeeters from hardware map
        yeeterLeft = hardwareMap.get(DcMotorEx.class, "yeeterLeft");
        yeeterRight = hardwareMap.get(DcMotorEx.class, "yeeterRight");

        // Reverse direction of right Yeeter
        yeeterRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        // Toggle yeeter active based on Y button and previous button state
        if(gamepad.yWasPressed()){
            yeeterActive = !yeeterActive;
        }

        // Update previous button state


        // Set yeeter powers if active or B button is pressed using proportional
        double yeeterLeftPower = 0;
        double yeeterRightPower = 0;
        if (yeeterActive || gamepad.b ) {
          yeeterLeftPower = (yeeterVelocityTarget - yeeterLeft.getVelocity()) * kP;
          yeeterRightPower = (yeeterVelocityTarget - yeeterRight.getVelocity()) * kP;
        }

        yeeterLeft.setPower(yeeterLeftPower);
        yeeterRight.setPower(yeeterRightPower);

        //add T
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
}
