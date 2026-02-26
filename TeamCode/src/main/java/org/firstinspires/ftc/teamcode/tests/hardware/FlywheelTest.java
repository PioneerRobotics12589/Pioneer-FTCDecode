package org.firstinspires.ftc.teamcode.tests.hardware;

import static org.firstinspires.ftc.teamcode.utility.Actuation.packet;
import static org.firstinspires.ftc.teamcode.utility.Actuation.voltageCompensation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.SimpleMotorFeedforward;

@TeleOp(name = "Flywheel Test", group = "tests")
@Config
public class FlywheelTest extends OpMode {
    DcMotorEx flywheel;
    DcMotor intake, transfer;
    public static int rpm;
    // 0.006
    public static double ks, kv, ka;
    public static double kp, ki, kd;
    public static PIDController PIDControl = new PIDController(kp, ki, kd);
    public static SimpleMotorFeedforward feedforwardControl = new SimpleMotorFeedforward(ks, kv, ka);
    public double error;

    public static double intakePower, transferPower;

    FtcDashboard dashboard;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        PIDControl = new PIDController(kp, ki, kd);
        feedforwardControl = new SimpleMotorFeedforward(ks, kv, ka);

//        double feedforward = 0.0;
        double feedforward = feedforwardControl.calculate(rpm);
//        double pid = 0.0;
        double pid = PIDControl.calculateSignal(rpm, flywheel.getVelocity());
        double signal = Math.max(-1, Math.min(1, voltageCompensation(feedforward + pid)));
        flywheel.setPower(signal);

        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        TelemetryPacket packet = new TelemetryPacket();

        error = rpm - flywheel.getVelocity();
        packet.put("target velocity", rpm);
        packet.put("actual velocity", flywheel.getVelocity());
        packet.put("flywheel signal clamped", signal);
        packet.put("flywheel signal unclamped", voltageCompensation(feedforward + pid));
        packet.put("error", error);
        dashboard.sendTelemetryPacket(packet);
    }
}