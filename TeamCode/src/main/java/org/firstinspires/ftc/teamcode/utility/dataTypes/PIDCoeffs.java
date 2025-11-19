package org.firstinspires.ftc.teamcode.utility.dataTypes;

public class PIDCoeffs {
    public double Kp, Ki, Kd;

    public PIDCoeffs(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public PIDCoeffs(double[] gains) {
        this.Kp = gains[0];
        this.Ki = gains[1];
        this.Kd = gains[2];
    }
}