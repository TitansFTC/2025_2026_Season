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

    public void loop(Gamepad gamepad, Telemetry telemetry, boolean yeetersReady) {
        //Activate feeder servos if x or b is pressed
        double feederAPower = 0;
        double feederBPower = 0;
        double feederCPower = 0;
        double feederDPower = 0;

        if (gamepad.a) {
            feederAPower = -0.8; // flipped during reconstruction
            feederBPower = -0.8;
            feederCPower = -0.8;
            feederDPower = -1; // Reverse to hold back balls
        }
        else if (gamepad.b) {
            feederAPower = -1; // flipped during reconstruction
            feederBPower = -1;
            feederCPower = -1;
            feederDPower = -1;
        }

        if (gamepad.b){
            feederDPower = 1;
        }

        feederA.setPower(feederAPower);
        feederB.setPower(feederBPower);
        feederC.setPower(feederCPower);
        feederD.setPower(feederDPower);

        //add telemetry
        telemetry.addData("feederAPower", feederAPower );
        telemetry.addData("feederBPower" , feederBPower);
        telemetry.addData("feederCPower" , feederCPower);
        telemetry.addData("feederDPower", feederDPower);
    }

    public void enableFeeders(boolean shoot) {
        if (shoot) {
            feederA.setPower(-1);
            feederB.setPower(-1);
            feederC.setPower(-1);
            feederD.setPower(1);
        }
        else{
            feederA.setPower(-0.8);
            feederB.setPower(-0.8);
            feederC.setPower(-0.8);
            feederD.setPower(-1);
        }
    }

    public void stop() {
        //stop all servos
        feederA.setPower(0);
        feederB.setPower(0);
        feederC.setPower(0);
        feederD.setPower(0);
    }
}
