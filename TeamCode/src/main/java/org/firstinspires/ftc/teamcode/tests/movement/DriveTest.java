package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(name="Drive Test", group="tests")
public class DriveTest extends OpMode {

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
        telemetry.addData("LeftStickY", gamepad1.left_stick_y);
        telemetry.addData("RightStickX", gamepad1.left_stick_y);
        telemetry.addData("LeftStickX", gamepad1.left_stick_y);
    }
}
