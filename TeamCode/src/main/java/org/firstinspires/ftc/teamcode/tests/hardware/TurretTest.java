package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.PIDController;

@TeleOp(name="Turret Test")
@Config
public class TurretTest extends OpMode {

    public static double targetAngle;
    public static int flywheelSpeed;
    public static double kp, ki, kd;
    private PIDController pid = new PIDController(kp, ki, kd);
    private DcMotorEx turret;

    private static FtcDashboard dash;
    private static TelemetryPacket packet;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
//      Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        pid = new PIDController(kp, ki, kd);
        double current = (double) turret.getCurrentPosition() / (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio);
        current %= (2 * Math.PI);
        double target = Math.toRadians(targetAngle) % (2 * Math.PI);

        if (current > target) {
            while (Math.abs(current - target) > Math.toRadians(180)) {
                target += 2 * Math.PI;
            }
        } else if (current < target) {
            while (Math.abs(current - target) > Math.toRadians(180)) {
                target -= 2 * Math.PI;
            }
        }

//        // Spin if the target is further than the maximum angle for either rotation
//        if (target > Math.toRadians(240)) {
//            target -= Math.toRadians(360);
//        } else if (target < -Math.toRadians(240)) {
//            target += Math.toRadians(360);
//        }

        double turretSignal = pid.calculateSignal(target, current);
        double clampedTurret = Math.max(-1.0, Math.min(1.0, turretSignal)); // Clamp signal between -1 & 1

        turret.setPower(clampedTurret);

        telemetry.addData("Turret Angle", Math.toDegrees(current));
        telemetry.addData("Turret Ticks", turret.getCurrentPosition());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Turret Signal", clampedTurret);
        packet.put("Turret Angle", Math.toDegrees(current));
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
