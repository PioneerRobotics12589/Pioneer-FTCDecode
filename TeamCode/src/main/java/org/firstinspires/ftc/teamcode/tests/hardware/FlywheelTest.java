package org.firstinspires.ftc.teamcode.tests.hardware;

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
    DcMotorEx flywheel1, flywheel2;
    public static int rpm;

    public static double kp1, ki1, kd1;
    public static double kp2, ki2, kd2;

    FtcDashboard dashboard;

    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp1, ki1, kd1, 0));

        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp2, ki2, kd2, 0));

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp1, ki1, kd1, 0));
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp2, ki2, kd2, 0));

        flywheel1.setVelocity(rpm);
        flywheel2.setVelocity(rpm);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("target rpm", rpm);
        packet.put("actual 1 rpm", flywheel1.getVelocity());
        packet.put("actual 2 rpm", flywheel2.getVelocity());
        dashboard.sendTelemetryPacket(packet);
    }
}