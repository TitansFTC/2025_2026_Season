// 1. Set F first... increase until it can hit target and minimize error
// 2. Set P second... increase until not overshooting and not toggle

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class YeetersTunerTeleop extends OpMode {
    private DcMotorEx yeeterLeft  = null;
    private DcMotorEx yeeterRight = null;

    private double[] yeeterVelocities = {0, 900, 1500};
    private int velocityIndex = 1;

    private double kP = 0;
    private double kF = 0;

    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 1;

    private FtcDashboard dashboard = null;
    private Telemetry    dashboardTelemetry = null;

    @Override
    public void init() {
    	// Set coefficients
	    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);

	    // Setup left Yeeter
    	yeeterLeft = hardwareMap.get(DcMotorEx.class, "yeeterLeft");
	    yeeterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	    yeeterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

	    // Setup right Yeeter
    	yeeterRight = hardwareMap.get(DcMotorEx.class, "yeeterRight");
	    yeeterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	    yeeterRight.setDirection(DcMotorEx.Direction.REVERSE);
	    yeeterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

	    // Get dashboard and telemetry instances
	    dashboard = FtcDashboard.getInstance();
	    dashboardTelemetry = dashboard.getTelemetry();

	    telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
    	// Y cycles forward through velocities, x cycles backwards
    	if (gamepad1.yWasPressed()) {
    	    velocityIndex = (velocityIndex + 1) % yeeterVelocities.length;
	    }
        if (gamepad1.xWasPressed()) {
            velocityIndex = Math.max(0, (velocityIndex - 1));
        }

	    // B cycles through step sizes
	    if (gamepad1.bWasPressed()) {
	        stepIndex = (stepIndex + 1) % stepSizes.length;
	    }

	    // Left decreases F, Right increases F
	    if (gamepad1.dpadLeftWasPressed()) {
	        kF -= stepSizes[stepIndex];
	    }

	    if (gamepad1.dpadRightWasPressed()) {
	        kF += stepSizes[stepIndex];
	    }

	    // Down decreases P, Up increases P
	    if (gamepad1.dpadDownWasPressed()) {
	        kP -= stepSizes[stepIndex];
	    }

	    if (gamepad1.dpadUpWasPressed()) {
	        kP += stepSizes[stepIndex];
	    }

    	// Set new coefficients
	    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);
	    yeeterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
	    yeeterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

	    // Set velocities
        double yeeterTargetVelocity = yeeterVelocities[velocityIndex];
	    yeeterLeft.setVelocity(yeeterTargetVelocity);
	    yeeterRight.setVelocity(yeeterTargetVelocity);

	    double yeeterLeftError = yeeterTargetVelocity - yeeterLeft.getVelocity();
	    double yeeterRightError = yeeterTargetVelocity - yeeterRight.getVelocity();

	    dashboardTelemetry.addData("yeeterTargetVelocity", yeeterTargetVelocity);
	    dashboardTelemetry.addData("yeeterLeftVelocity", yeeterLeft.getVelocity());
	    dashboardTelemetry.addData("yeeterRightVelocity", yeeterRight.getVelocity());
	    dashboardTelemetry.addData("yeeterLeftError", yeeterLeftError);
	    dashboardTelemetry.addData("yeeterRightError", yeeterRightError);
	    dashboardTelemetry.addData("kP", kP);
	    dashboardTelemetry.addData("kF", kF);
	    dashboardTelemetry.addData("step size", stepSizes[stepIndex]);
	    dashboardTelemetry.update();
    }
}
