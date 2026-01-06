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
        double launcherFeederPower = 0;

        if (gamepad.x || gamepad.b) {
            feederPower = -1; //flipped during reconstruction
        }

        if (gamepad.b){
            launcherFeederPower = 1;
        }

        feederA.setPower(feederPower);
        feederB.setPower(feederPower);
        feederC.setPower(feederPower);
        feederD.setPower(launcherFeederPower);

        //add telemetry
        telemetry.addData("feederPower", feederPower);
    }

    public void enableFeeders() {
        feederA.setPower(-1);
        feederB.setPower(-1);
        feederC.setPower(-1);
        feederD.setPower(1);
    }

    public void stop() {
        //stop all servos
        feederA.setPower(0);
        feederB.setPower(0);
        feederC.setPower(0);
        feederD.setPower(0);
    }
}
