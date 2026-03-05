package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeders {
    private static final double kP = 150;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 10;
    private static final double CHAIN_FAST_POWER = -1000;
    private static final double CHAIN_SLOW_POWER = -100;

    private CRServo feederTop = null;
    private DcMotorEx feederBottom = null;

    public Feeders(HardwareMap hardwareMap) {
        feederTop = hardwareMap.get(CRServo.class, "feederTop");

        feederBottom = hardwareMap.get(DcMotorEx.class, "feederBottom");
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);
        feederBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        feederBottom.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        feederBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry, boolean yeetersReady) {
        //Activate feeder servos if x or b is pressed
        double feederTopPower = 0;
        double feederBottomVelocity = 0;

        if (gamepad.a) {
            feederTopPower = -1; // flipped during reconstruction
            feederBottomVelocity = CHAIN_SLOW_POWER;
        }
        else if (gamepad.b) {
            feederTopPower = 1; // flipped during reconstruction
            feederBottomVelocity = CHAIN_FAST_POWER;
        }

        feederTop.setPower(feederTopPower);
        feederBottom.setVelocity(feederBottomVelocity);

        //add telemetry
        telemetry.addData("feederTopPower", feederTopPower );
        telemetry.addData("feederBottomVelocity" , feederBottomVelocity);
    }

    public void enableFeeders(boolean shoot) {
        if (shoot) {
            feederTop.setPower(1);
            feederBottom.setVelocity(CHAIN_FAST_POWER);
        }
        else{
            feederTop.setPower(-1);
            feederBottom.setVelocity(CHAIN_SLOW_POWER);
        }
    }

    public void stop() {
        //stop all servos
        feederTop.setPower(0);
        feederBottom.setPower(0);
    }
}
