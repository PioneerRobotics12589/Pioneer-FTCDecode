package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.SimpleMotorFeedforward;

@TeleOp(name="Turret Alternative Test")
@Config
public class TurretAlternativeTest extends OpMode {
    public static double targetAngle;
    public static double turretPower;
    public static double kp, ki, kd, ks;
    private PIDController pid = new PIDController(kp, ki, kd);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.07, -0.0000230769);
    private double accel_max = 15000;
    private double vel_max = 15000;
    private CRServo turret;
    private DcMotorEx intake;
    private double lastTime;
    private double lastTicks;
    private double lastVel;
    private static FtcDashboard dash;
    private static TelemetryPacket packet;

    @Override
    public void init() {
        turret = hardwareMap.crservo.get("turret");
//        turret.setTargetPosition(0);
//        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        turret.setPower(1.0);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        Actuation.setup(hardwareMap, telemetry);

        pid = new PIDController(kp, ki, kd);
        lastTime = System.nanoTime();
//        Actuation.setTurret(Math.toRadians(10));
    }

    @Override
    public void loop() {
        pid = new PIDController(kp, ki, kd);
        double current = intake.getCurrentPosition() / (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio);
        current = AngleUnit.normalizeRadians(current + OttoCore.robotPose.heading);

        double target = AngleUnit.normalizeRadians(Math.toRadians(targetAngle));
        target = Math.max(-ActuationConstants.Launcher.turretMaxAngle, Math.min(ActuationConstants.Launcher.turretMaxAngle, target));

        double turretSignal = pid.calculateSignal(target, current);
////        double turretFF = (Math.abs(target - current) > Math.toRadians(0.5)) ? Math.signum(turretSignal) * ks : 0.0;
        double clampedTurret = Math.max(-1.0, Math.min(1.0, turretSignal)); // Clamp signal between -1 & 1
//
        turret.setPower(clampedTurret * turretPower);

//        telemetry.addData("Turret Ticks", turret.getCurrentPosition());
//        telemetry.addData("Target Angle", Math.toDegrees(target));
        packet.put("Turret Angle", Math.toDegrees(current));
        packet.put("Target Angle", AngleUnit.normalizeDegrees(targetAngle));
        packet.put("Signal", clampedTurret);
        packet.put("Turret", turretPower);
        packet.put("Position", Math.toDegrees(current));
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
