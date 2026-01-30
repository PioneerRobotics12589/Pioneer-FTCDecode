package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Light Test", group="tests")
public class LightTest extends OpMode {

    public static Servo light;
    public static double color = 0;

    @Override
    public void init() {
        light = hardwareMap.get(Servo.class, "light");
    }

    @Override
    public void loop() {
        light.setPosition(color);
        color += 0.0005;
        if (color >= 0.8) {
            color = 0.3;
        }
    }
}
