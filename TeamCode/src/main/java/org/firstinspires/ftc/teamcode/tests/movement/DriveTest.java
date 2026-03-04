package org.firstinspires.ftc.teamcode.tests.movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@Config
@TeleOp(name="Drive Test", group="tests")
public class DriveTest extends OpMode {

    public static double move, strafe, turn;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Actuation.drive(move, turn, strafe);
        telemetry.addData("LeftStickY", gamepad1.left_stick_y);
        telemetry.addData("RightStickX", gamepad1.left_stick_y);
        telemetry.addData("LeftStickX", gamepad1.left_stick_y);
    }
}
