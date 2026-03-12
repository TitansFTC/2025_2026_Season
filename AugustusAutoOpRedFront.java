package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="AugustusAutoOpRedFront", group = "Titan's AutoOp")
public class AugustusAutoOpRedFront extends LinearOpMode {

    private DriveBase driveBase = null;
    private Intake intake = null;
    private Feeders feeders = null;
    private Yeeters yeeters = null;

    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;

    private Webcam webcam = null;

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

        // Activate Yeeters and move backward
        yeeters.toggleYeeters();

        driveBase.moveBackward(driveBase.HALF_POWER_FRACTION, 1750);
        driveBase.stop();
        sleep(250);

        // Turn on Yeeters
        timer.reset();
        while (timer.seconds() < 0.75) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance()); // Big number to shoot far
            dashboardTelemetry.update();
        }

        // Turn on Feeders
        feeders.enableFeeders(true);
        intake.enableIntake();
        timer.reset();

        while (timer.seconds() < 2.25) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance());
            dashboardTelemetry.update();
        }
        yeeters.toggleYeeters();

        // 3 Pt Turn: Rotate, move back, finish rotation
        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 350);
        driveBase.stop();
        sleep(250);

        driveBase.moveBackward(driveBase.HALF_POWER_FRACTION, 150);
        driveBase.stop();
        sleep(250);

        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 850);
        driveBase.stop();
        sleep(250);

        //Turn Intake On
        //intake.enableIntake();
        feeders.enableFeeders(false);

        //Move
        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 1500);
        driveBase.stop();
        sleep(1000);

        // Move Forward
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 300);
        feeders.stop();
        intake.stop();

        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 800);
        driveBase.stop();
        sleep(250);

        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 1250);
        driveBase.stop();
        sleep(250);

        // Turn yeeters back on, begin feeding
        yeeters.toggleYeeters();
        timer.reset();
        while (timer.seconds() < 0.75) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance()); // Big number to shoot far
            dashboardTelemetry.update();
        }
        feeders.enableFeeders(true);
        intake.enableIntake();

        timer.reset();
        while (timer.seconds() < 2.25) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance());
            dashboardTelemetry.update();
        }
        yeeters.toggleYeeters();
        feeders.stop();
        intake.stop();

        driveBase.strafeLeft(driveBase.HALF_POWER_FRACTION, 1000);
        driveBase.stop();
        sleep(250);

        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 1000);
        sleep(250);
        feeders.enableFeeders(false);
        intake.enableIntake();

        driveBase.strafeRight(driveBase.HALF_POWER_FRACTION, 700);
        driveBase.stop();
        sleep(250);

        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 975);
        driveBase.stop();
        sleep(250);

        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 625);
        driveBase.stop();

        feeders.stop();
        intake.stop();
    }
}