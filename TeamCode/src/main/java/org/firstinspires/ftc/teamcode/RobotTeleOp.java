package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(name = "Awe(sigma) Sauce")
@Config
public class RobotTeleOp extends OpMode {

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    public void loop() {
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

        if (gamepad1.right_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);
        }
        else if (gamepad1.left_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);
        }
        else {
            Actuation.setFlywheel(0);
            Actuation.checkFlywheelSpeed(gamepad1, 0);
        }

        Actuation.setIntake(gamepad1.dpad_down);
        Actuation.runBackwards(gamepad1.right_trigger > 0.5);
        Actuation.setTransfer(gamepad1.left_trigger > 0.5);
    }
}