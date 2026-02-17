package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="AugustusAutoOpBlueBack", group = "Titan's AutoOp")
public class AugustusAutoOpBlueBack extends LinearOpMode {

    private DriveBase driveBase = null;
    private Intake intake = null;
    private Feeders feeders = null;
    private Yeeters yeeters = null;
    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;

    private Webcam webcam = null;

    private double targetDistance = 140;

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

        //Move to good shooting distance
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 220);
        driveBase.stop();

        //Rotate toward the goal
        driveBase.rotateLeft(driveBase.SLOW_POWER_FRACTION, 35);
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
        //Move to collect the balls
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 620);
        //Turn turn balls
        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 1100);
        //pick up balls
        feeders.enableFeeders(false);
        //Move back
        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 750);

        feeders.stop();

        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 400);

        driveBase.stop();
        feeders.stop();
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 1300);
        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 825);
        driveBase.moveBackward(driveBase.HALF_POWER_FRACTION, 400);
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