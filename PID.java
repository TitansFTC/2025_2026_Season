package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    private double lastError = 0;
    private double integralSum = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double target, double current) {
        double error = target - current;
        double derivative = (error - lastError) / timer.seconds();

        integralSum += error * timer.seconds();
        lastError = error;

        return (error * kP) + (integralSum * kI) + (derivative * kD);
    }
}
