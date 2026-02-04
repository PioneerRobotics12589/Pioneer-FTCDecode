package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;

@TeleOp(name="Turret Test")
@Config
public class TurretTest extends OpMode {

    public static double targetAngle;
    private DcMotor turret;

    public static double turretPower;

    private static FtcDashboard dash;
    private static TelemetryPacket packet;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(turretPower);

        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        turret.setPower(turretPower);
        double current = turret.getCurrentPosition() / (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio);

        double currentNorm = AngleUnit.normalizeRadians(current);
        double targetNorm = AngleUnit.normalizeRadians(Math.toRadians(targetAngle));

        targetNorm = Math.max(-ActuationConstants.Launcher.turretMaxAngle, Math.min(targetNorm, ActuationConstants.Launcher.turretMaxAngle));

        int targetTicks = (int) (targetNorm * (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio));

        turret.setTargetPosition(targetTicks);

        telemetry.addData("Turret Angle", Math.toDegrees(current));
        telemetry.addData("Turret Ticks", turret.getCurrentPosition());
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Target Angle", Math.toDegrees(targetNorm));
        packet.put("Turret Angle", Math.toDegrees(current));
        packet.put("Target Angle", Math.toDegrees(targetNorm));
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
