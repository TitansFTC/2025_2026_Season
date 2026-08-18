package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="OdometryAutoTest", group = "Titan's AutoOp")
public class OdometryAutoTest extends LinearOpMode {

    private DriveBase driveBase = null;
    private Intake intake = null;
    private Feeders feeders = null;
    private Yeeters yeeters = null;
    private Odometry odometry = null;
    private Webcam webcam = null;

    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;

    private double targetDistance = 155;

    public void initHardware() {
        //Initialize DB, I, F, Y
        driveBase = new DriveBase(hardwareMap);
        intake = new Intake(hardwareMap);
        feeders = new Feeders(hardwareMap);
        yeeters = new Yeeters(hardwareMap);
        webcam = new Webcam(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap);

        //Get dashboard and T instances
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        double cur_pos_x;
        double cur_pos_y;
        double cur_angle;

        while (opModeIsActive()) {
            odometry.loop(dashboardTelemetry);
            cur_pos_x = odometry.cur_pos_x();
            cur_pos_y = odometry.cur_pos_y();
            cur_angle = odometry.cur_angle();

            double[] powers = driveBase.odoCompute(0, -12, 0, cur_pos_x, cur_pos_y, cur_angle);
            driveBase.odoMove(0, -12, 0, cur_pos_x, cur_pos_y, cur_angle);

            dashboardTelemetry.addData("leftFrontPower", powers[0]);
            dashboardTelemetry.addData("leftBackPower", powers[1]);
            dashboardTelemetry.addData("rightFrontPower", powers[2]);
            dashboardTelemetry.addData("rightBackPower", powers[3]);
            dashboardTelemetry.addData("powerFraction", powers[4]);
            dashboardTelemetry.addData("cur_pos_x", cur_pos_x );
            dashboardTelemetry.addData("cur_pos_y", cur_pos_y );
            dashboardTelemetry.addData("cur_angle", cur_angle );

            dashboardTelemetry.update();
        }
    }
}