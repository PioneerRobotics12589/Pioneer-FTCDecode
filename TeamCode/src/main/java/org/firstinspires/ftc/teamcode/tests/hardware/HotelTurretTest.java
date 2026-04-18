package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hotel Turret Test")
@Config
public class HotelTurretTest extends OpMode {
    Servo servo;
    public static double servoAngle;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "turret");
    }

    @Override
    public void loop() {
        double servoAngleNorm = servoAngle / 90.0;
        servo.setPosition(servoAngleNorm);
    }
}
