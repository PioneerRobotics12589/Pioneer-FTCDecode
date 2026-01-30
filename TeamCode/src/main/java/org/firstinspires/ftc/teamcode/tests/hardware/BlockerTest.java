package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Blocker Test", group = "tests")
public class BlockerTest extends OpMode {
    public static double angle;
    private Servo blocker;

    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, "blocker");
    }

    @Override
    public void loop() {
        blocker.setPosition(angle);
    }
}
