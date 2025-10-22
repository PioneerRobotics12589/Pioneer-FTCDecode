package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name="Starter Robot (Awe∑ Sauce)")
@Config
public class StarterRobot extends OpMode {
    FtcDashboard dashboard;
    public static int flywheelVel = 0;
    public static boolean isTesting = true, servos = false;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();
    }

    public void loop() {
        Actuation.drive(gamepad1.left_stick_y, gamepad1.right_stick_x, 0.0);

        if (!isTesting) {
            if (gamepad1.right_bumper) {
                Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);
            } else if (gamepad1.left_bumper) {
                Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);
            }
            Actuation.setFlywheel(0);
            Actuation.checkFlywheelSpeed(gamepad1, 0);
            Actuation.setLoaders(gamepad1.right_trigger > 0.5);
        } else {
            Actuation.setFlywheel(flywheelVel);
            if (Math.abs(Actuation.flywheel.getVelocity() - flywheelVel) <= 20) {
                Actuation.packet.addLine("READY TO SHOOT");
            } else {
                Actuation.packet.addLine("FLYWHEEL NOT READY");
            }
            Actuation.updateTelemetry();
            Actuation.setLoaders(servos);
        }
    }
}