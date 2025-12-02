package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeders {
    private CRServo feederA = null;
    private CRServo feederB = null;
    private CRServo feederC = null;
    private CRServo feederD = null;

    public Feeders(HardwareMap hardwareMap) {
        feederA = hardwareMap.get(CRServo.class, "feederA");
        feederB = hardwareMap.get(CRServo.class, "feederB");
        feederC = hardwareMap.get(CRServo.class, "feederC");
        feederD = hardwareMap.get(CRServo.class, "feederD");
    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        //Activate feeder servos if x or b is pressed
        double feederPower = 0;
        double launcerFeederPower = 0;
        if (gamepad.x || gamepad.b) {
            feederPower = -1; //flipped during reconstruction
        }
        if (gamepad.b){
            launcerFeederPower = -1;
        }
        feederA.setPower(feederPower);
        feederB.setPower(feederPower);
        feederC.setPower(feederPower);
        feederD.setPower(launcerFeederPower);

        //add telemetry
        telemetry.addData("feederPower", feederPower);
    }

    public void stop() {
        //stop all servos
        feederA.setPower(0);
        feederB.setPower(0);
        feederC.setPower(0);
    }
}
