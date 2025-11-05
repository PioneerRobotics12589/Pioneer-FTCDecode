package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name = "Awe(sigma) Sauce")
public class RobotTeleOp extends OpMode {
    FtcDashboard dashboard;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();
    }

    public void loop() {
        Actuation.driveStarter(gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (gamepad1.right_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);
        } else if (gamepad1.left_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);
        }
        else {
            Actuation.setFlywheel(0);
            Actuation.checkFlywheelSpeed(gamepad1, 0);
        }

        if (gamepad1.left_trigger > 0.5) Actuation.setIntake(1.0);
        else Actuation.setIntake(0.0);

        Actuation.setLoaders(gamepad1.right_trigger > 0.5);
    }
}
