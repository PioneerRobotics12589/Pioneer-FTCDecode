package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Localization Test")
public class LocalizationTest extends OpMode {
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

//        Actuation.packet.put("ticks left", OttoCore.ticks_left);
//        Actuation.packet.put("ticks right", OttoCore.ticks_right);
//        Actuation.packet.put("ticks back", OttoCore.ticks_back);
        Actuation.packet.put("Pos", OttoCore.robotPose.toString());
        Actuation.packet.put("Vel", OttoCore.getVelocity().toString());

        Actuation.updateTelemetry();
    }
}