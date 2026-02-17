package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="AugustusAutoOpRedBack", group = "Titan's AutoOp")
public class AugustusAutoOpRedBack extends LinearOpMode {

    private DriveBase driveBase = null;
    private Intake intake = null;
    private Feeders feeders = null;
    private Yeeters yeeters = null;

    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;

    private Webcam webcam = null;

    private double targetDistance = 160;

    public void initHardware() {
        //Initialize DB, I, F, Y
        driveBase = new DriveBase(hardwareMap);
        intake = new Intake(hardwareMap);
        feeders = new Feeders(hardwareMap);
        yeeters = new Yeeters(hardwareMap);
        webcam = new Webcam(hardwareMap, telemetry);

        //Get dashboard and T instances
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 220);
        driveBase.stop();

        driveBase.rotateRight(driveBase.SLOW_POWER_FRACTION, 310);
        driveBase.stop();

        // Activate Yeeters and move forward
        yeeters.toggleYeeters();

        // Turn on Yeeters
        timer.reset();
        while (timer.seconds() < 3.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, targetDistance); // Big number to shoot far
            dashboardTelemetry.update();
        }

        // Turn on Feeders
        feeders.enableFeeders(true);
        intake.enableIntake();
        timer.reset();
        while (timer.seconds() < 7.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, targetDistance);
            dashboardTelemetry.update();
        }

        // Turn off Yeeters and Feeders
        yeeters.toggleYeeters();
        feeders.stop();

        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 650);
        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 740);
        feeders.enableFeeders(false);
        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 850);
        feeders.stop();
        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 400);

        driveBase.stop();
        feeders.stop();
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 1150);
        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 1150);
        driveBase.moveBackward(driveBase.HALF_POWER_FRACTION, 375);
        driveBase.stop();
        timer.reset();

        yeeters.toggleYeeters();
        sleep(1000);
        feeders.enableFeeders(true);

        while (timer.seconds() < 7.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, targetDistance);
            dashboardTelemetry.update();
        }
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 500);
        driveBase.stop();



    }
}