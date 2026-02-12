package org.firstinspires.ftc.teamcode.tests.hardware;

import static org.firstinspires.ftc.teamcode.utility.Actuation.flywheel1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel Test", group = "tests")
@Config
public class FlywheelTest extends OpMode {
    DcMotorEx flywheel;
    public static int rpm;

    public static double kp, ki, kd, kf;
    public static double error;

    FtcDashboard dashboard;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, kf));

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        //flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, kf));

        flywheel.setVelocity(rpm);

        TelemetryPacket packet = new TelemetryPacket();

        error = rpm - flywheel.getVelocity();
        packet.put("target velocity", rpm);
        packet.put("actual velocity", flywheel.getVelocity());
        packet.put("error", error);
        dashboard.sendTelemetryPacket(packet);
    }
}