package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AugustusTeleop", group="Titans TeleOps")

public class AugustusTeleop extends OpMode {
    private DriveBase driveBase = null;
    private Intake intake = null;
    private Feeders feeders = null;
    private Yeeters yeeters = null;

    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;

    private Webcam webcam = null;

    @Override
    public void init() {
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
    public void loop() {
        // Loop drive base with gamepad 1
        driveBase.loop(gamepad1, dashboardTelemetry);

        // Loop IFY with gamepad2
        intake.loop(gamepad2, dashboardTelemetry);
        feeders.loop(gamepad2, dashboardTelemetry);
        yeeters.loop(gamepad2, dashboardTelemetry);

        //Update A tags
        webcam.loop(dashboardTelemetry);
        //update dashboard
        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        // Stop DIFY
        driveBase.stop();
        intake.stop();
        feeders.stop();

        yeeters.stop();
    }
}

