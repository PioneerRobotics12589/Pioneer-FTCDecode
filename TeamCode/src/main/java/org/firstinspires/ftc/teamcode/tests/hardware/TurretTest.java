package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;

@TeleOp(name="Turret Test")
@Config
public class TurretTest extends OpMode {

    public static double targetAngle;
    private DcMotor turret;

    private static FtcDashboard dash;
    private static TelemetryPacket packet;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.0);

        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        double current = turret.getCurrentPosition() / (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio);
        current = (current + Math.PI * 2.0) % (2 * Math.PI);

        double target = (Math.toRadians(targetAngle) + Math.PI * 2.0) % (2 * Math.PI);

        if (current > target) {
            while (Math.abs(current - target) > Math.toRadians(180)) {
                target += 2*Math.PI;
            }
        } else if (current < target) {
            while (Math.abs(current - target) > Math.toRadians(180)) {
                target -= 2*Math.PI;
            }
        }

        if (target > ActuationConstants.Launcher.turretMaxAngle) {
            target = -ActuationConstants.Launcher.turretMaxAngle;
        } else if (target < -ActuationConstants.Launcher.turretMaxAngle) {
            target = ActuationConstants.Launcher.turretMaxAngle;
        }

        int targetTicks = (int) (target * (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio));

        turret.setTargetPosition(targetTicks);

        telemetry.addData("Turret Angle", Math.toDegrees(current));
        telemetry.addData("Turret Ticks", turret.getCurrentPosition());
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Target Angle", Math.toDegrees(target));
        packet.put("Turret Angle", Math.toDegrees(current));
        packet.put("Target Angle", Math.toDegrees(target));
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
