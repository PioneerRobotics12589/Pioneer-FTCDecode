package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDCoeffs;

import java.util.concurrent.TimeUnit;

public class PIDController {
    double prevError;
    public double error;

    double Kp, Ki, Kd;

    double integral, derivative;

    long lastTime, startTime;

    double prevTarget;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        prevError = 0.0;

        integral = 0;
        error = 0;
        derivative = 0;

        startTime = TimeUnit.SECONDS.convert(System.nanoTime(), TimeUnit.NANOSECONDS);
        lastTime = System.nanoTime();
    }

    public PIDController(PIDCoeffs gains) {
        this.Kp = gains.Kp;
        this.Ki = gains.Ki;
        this.Kd = gains.Kd;

        prevError = 0.0;

        integral = 0;
        error = 0;
        derivative = 0;

        startTime = TimeUnit.SECONDS.convert(System.nanoTime(), TimeUnit.NANOSECONDS);
        lastTime = System.nanoTime();
    }

    public void updateCoeffs(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void updateCoeffs(PIDCoeffs gains) {
        this.Kp = gains.Kp;
        this.Ki = gains.Ki;
        this.Kd = gains.Kd;
    }

    public double calculateSignal(double target, double current) {
        double dt = (System.nanoTime() - lastTime)/1000000000.00;

        error = target-current;

        integral = (error * dt); // NEEDS TO BE FIXED TO integral += error * dt;
        derivative = (error - prevError) / dt;

        double p = Kp * error;
        double i = Ki * integral;
        double d = Kd * derivative;

        lastTime = System.nanoTime();
        prevError = error;

        return (p + i + d);
    }
}