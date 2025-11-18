package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotorEx intake = null;

    public Intake(HardwareMap hardwareMap) {
        //Get intake from hardware map
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        //reverse direction of motor
        intake.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        //activate intake if a or b is pressed
        double intakePower = 0;
        if (gamepad.a || gamepad.b) {
            intakePower = 1;
        }

        intake.setPower(intakePower);

        //add telemetry
        telemetry.addData("intakePower", intakePower);
    }

    public void stop() {
        //stop all motors
        intake.setPower(0);
    }

}
