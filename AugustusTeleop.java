package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AugustusTeleop", group="Titans TeleOps")

public class AugustusTeleop extends OpMode {
    private DriveBase driveBase = null;
    private Intake intake = null;
    private Feeders feeders = null;
    private Yeeters yeeters = null;
    //private Odometry odometry = null;

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
        //odometry = new Odometry(hardwareMap);

        //Get dashboard and T instances
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Loop webcam, odometry (optional), drive base with gamepad 1
        webcam.loop(dashboardTelemetry);

        double cur_pos_x = 0;
        double cur_pos_y = 0;
        double cur_angle = 0;

        /*
        if (gamepad1.left_trigger>.8){
            odometry.loop(dashboardTelemetry);
            cur_pos_x = odometry.cur_pos_x();
            cur_pos_y = odometry.cur_pos_y();
            cur_angle = odometry.cur_angle();
        }
        */

        driveBase.loop(gamepad1, dashboardTelemetry, webcam.getTargetBearing(), cur_pos_x, cur_pos_y, cur_angle);

        // Loop IFY with gamepad2
        intake.loop(gamepad2, dashboardTelemetry);
        feeders.loop(gamepad2, dashboardTelemetry, yeeters.isReady());
        yeeters.loop(gamepad2, dashboardTelemetry, webcam.getTargetDistance());

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

