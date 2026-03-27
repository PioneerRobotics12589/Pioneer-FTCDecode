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
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.SimpleMotorFeedforward;

@TeleOp(name = "Flywheel Test", group = "tests")
@Config
public class FlywheelTest extends OpMode {
    DcMotorEx flywheel, flywheel1;
    DcMotor intake, transfer;
    public static int rpm, rpm1;
    public static double kp = 500, ki = 0, kd = 0, kf = 17;
    // 0.006
//    public static double ks = 0.006, kv = 0.00039, ka = 0.0;
//    public static double kp = 0.65, ki = 0.0, kd = 0.0002;
//    public static PIDController PIDControl = new PIDController(kp, ki, kd);
//    public static SimpleMotorFeedforward feedforwardControl = new SimpleMotorFeedforward(ks, kv, ka);
    public double error;

    public static double intakePower, transferPower;

    FtcDashboard dashboard;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        OttoCore.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
//        PIDControl = new PIDController(kp, ki, kd);
//        feedforwardControl = new SimpleMotorFeedforward(ks, kv, ka);

//        double feedforward = 0.0;
//        double feedforward = feedforwardControl.calculate(rpm);
//        double pid = 0.0;
//        double pid = PIDControl.calculateSignal(rpm, flywheel.getVelocity());
//        double signal = Math.max(0, Math.min(1, voltageCompensation(feedforward + pid)));
//        flywheel.setPower(signal);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, kf));
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, kf));

        flywheel.setVelocity(rpm);
        flywheel1.setVelocity(rpm);

        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        TelemetryPacket packet = new TelemetryPacket();

        error = rpm - flywheel.getVelocity();
        packet.put("target velocity", rpm);
        packet.put("actual velocity", flywheel.getVelocity());
        packet.put("actual velocity1", flywheel1.getVelocity());
//        packet.put("flywheel signal clamped", signal);
//        packet.put("flywheel signal unclamped", voltageCompensation(feedforward + pid));
        packet.put("error", error);
        dashboard.sendTelemetryPacket(packet);
    }
}