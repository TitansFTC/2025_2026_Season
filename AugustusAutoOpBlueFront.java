package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="AugustusAutoOpBlueFront", group = "Titan's AutoOp")
public class AugustusAutoOpBlueFront extends LinearOpMode {

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

        driveBase.moveBackward(driveBase.HALF_POWER_FRACTION, 1800);
        driveBase.stop();
        sleep(500);

        driveBase.rotateRight(driveBase.SLOW_POWER_FRACTION, 75);
        driveBase.stop();
        sleep(250);

        // Turn on Yeeters
        timer.reset();
        while (timer.seconds() < 1.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance()); // Big number to shoot far
            dashboardTelemetry.update();
        }

        // Turn on Feeders
        feeders.enableFeeders(true);
        intake.enableIntake();
        timer.reset();

        while (timer.seconds() < 3.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance());
            dashboardTelemetry.update();
        }
        yeeters.toggleYeeters();

        // 3 Pt Turn: Rotate, move back, finish rotation
        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 300);
        driveBase.stop();
        sleep(250);

        driveBase.moveBackward(driveBase.HALF_POWER_FRACTION, 175);
        driveBase.stop();
        sleep(250);

        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 750);
        driveBase.stop();
        sleep(250);

        //Turn Intake On
        //intake.enableIntake();
        feeders.enableFeeders(false);

        //Move
        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 1600);
        driveBase.stop();
        sleep(250);

        // Move Forward
        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 300);
        feeders.stop();
        intake.stop();

        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 750);
        driveBase.stop();
        sleep(250);

        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 1000);
        driveBase.stop();
        sleep(250);

        // Turn on Yeeters
        yeeters.toggleYeeters();
        timer.reset();
        while (timer.seconds() < 1.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance()); // Big number to shoot far
            dashboardTelemetry.update();
        }
        feeders.enableFeeders(true);
        intake.enableIntake();

        timer.reset();
        while (timer.seconds() < 3.0) {
            webcam.loop(dashboardTelemetry);
            yeeters.loop(gamepad1, dashboardTelemetry, webcam.getTargetDistance());
            dashboardTelemetry.update();
        }
        yeeters.toggleYeeters();
        feeders.stop();
        intake.stop();

        driveBase.strafeRight(driveBase.HALF_POWER_FRACTION, 750);
        driveBase.stop();
        sleep(250);

        driveBase.rotateRight(driveBase.HALF_POWER_FRACTION, 900);
        sleep(250);
        feeders.enableFeeders(false);
        intake.enableIntake();

        driveBase.strafeLeft(driveBase.HALF_POWER_FRACTION, 900);
        driveBase.stop();
        sleep(250);

        driveBase.moveBackward(driveBase.SLOW_POWER_FRACTION, 950);
        driveBase.stop();
        sleep(250);

        driveBase.rotateLeft(driveBase.HALF_POWER_FRACTION, 625);
        driveBase.stop();
        sleep(250);

        driveBase.moveForward(driveBase.HALF_POWER_FRACTION, 250);
        driveBase.stop();

        feeders.stop();
        intake.stop();
    }
}



