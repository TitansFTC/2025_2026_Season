package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="AugustusAutoOpRedBack", group = "Titan's AutoOp")
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

        // Activate Yeeters and move forward
        yeeters.toggleYeeters();

        driveBase.moveForward();
        while (timer.seconds() < 1.5) {
            sleep(1);
        }
        driveBase.stop();
        timer.reset();

        driveBase.rotateRight();
        while (timer.seconds() < 0.5) {
            sleep(1);
        }
        driveBase.stop();
        timer.reset();

        // Turn on Yeeters
        while (timer.seconds() < 3.0) {
            yeeters.loop(gamepad1, dashboardTelemetry, 500); // Big number to shoot far
            dashboardTelemetry.update();
        }
        timer.reset();

        // Turn on Feeders
        feeders.enableFeeders();
        while (timer.seconds() < 10.0) {
            yeeters.loop(gamepad1, dashboardTelemetry, 500);
            dashboardTelemetry.update();
        }
        timer.reset();

        // Turn off Yeeters and Feeders
        yeeters.toggleYeeters();
        feeders.stop();

        driveBase.moveForward();
        sleep(2000);
        driveBase.stop();
    }
}



