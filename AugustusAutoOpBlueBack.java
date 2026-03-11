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

    private double targetDistance = 155;

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

        // Activate Yeeters and move forward
        yeeters.toggleYeeters();

        // Move to good shooting distance
        driveBase.moveForward(driveBase.SLOW_POWER_FRACTION, 350);
        driveBase.stop();
        sleep(250);

        // Rotate toward the goal
        driveBase.rotateLeft(driveBase.SLOW_POWER_FRACTION, 215);
        driveBase.stop();
        sleep(250);

        // Warmup Yeeters
        timer.reset();
        while (timer.seconds() < 2.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, targetDistance); // Big number to shoot far
            dashboardTelemetry.update();
        }

        // Turn on Feeders
        feeders.enableFeeders(true);
        intake.enableIntake();

        // Shoot first volley
        timer.reset();
        while (timer.seconds() < 3.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, targetDistance);
            dashboardTelemetry.update();
        }

        // Turn off Feeders and Yeeters
        feeders.stop();
        intake.stop();
        yeeters.toggleYeeters();

        //Move to collect the balls
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 500);
        driveBase.stop();
        sleep(250);

        //Turn turn balls
        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 1000);
        driveBase.stop();
        sleep(250);

        //pick up balls
        feeders.enableFeeders(false);
        intake.enableIntake();

        //Move back
        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 1200);
        driveBase.stop();
        sleep(250);

        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 300);
        driveBase.stop();
        sleep(250);

        feeders.stop();
        intake.stop();

        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 550);
        driveBase.stop();
        sleep(250);

        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 875);
        driveBase.stop();
        sleep(250);

        driveBase.moveBackward(driveBase.HALF_POWER_FRACTION, 215);
        driveBase.stop();
        sleep(250);

        // Shoot second volley
        yeeters.toggleYeeters();
        timer.reset();
        while (timer.seconds() < 2.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, targetDistance); // Big number to shoot far
            dashboardTelemetry.update();
        }
        feeders.enableFeeders(true);
        intake.enableIntake();

        timer.reset();
        while (timer.seconds() < 3.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, targetDistance);
            dashboardTelemetry.update();
        }
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 450);
        driveBase.stop();
    }
}