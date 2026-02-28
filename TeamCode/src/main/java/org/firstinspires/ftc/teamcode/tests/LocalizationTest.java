package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.localization.PinpointControl;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="Localization Test")
public class LocalizationTest extends OpMode {

    public static double xOffset = -4.35, yOffset = -6.0;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        Actuation.packet.put("Pos", OttoCore.robotPose.toString());
        Actuation.packet.put("Vel", OttoCore.getVelocity().toString());
        Actuation.packet.put("VelX", OttoCore.getVelocity().x);
        Actuation.packet.put("VelY", OttoCore.getVelocity().y);
        Actuation.packet.put("VelHead", OttoCore.getVelocity().heading);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();
        PinpointControl.setOffsets(xOffset, yOffset);
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
        Actuation.packet.put("Move",gamepad1.left_stick_y);
        Actuation.packet.put("Strafe", -gamepad1.left_stick_x);
        Actuation.packet.put("Turn", -gamepad1.right_stick_x);

//        Actuation.packet.put("ticks left", OttoCore.ticks_left);
//        Actuation.packet.put("ticks right", OttoCore.ticks_right);
//        Actuation.packet.put("ticks back", OttoCore.ticks_back);
        Actuation.packet.put("Pos", OttoCore.robotPose.toString());
        Actuation.packet.put("Vel", OttoCore.getVelocity().toString());
        Actuation.packet.put("VelX", OttoCore.getVelocity().x);
        Actuation.packet.put("VelY", OttoCore.getVelocity().y);
        Actuation.packet.put("VelHead", OttoCore.getVelocity().heading);

        Actuation.updateTelemetry();
    }
}