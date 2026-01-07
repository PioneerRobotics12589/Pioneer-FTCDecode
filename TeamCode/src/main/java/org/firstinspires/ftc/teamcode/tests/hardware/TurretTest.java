package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(name="Turret Test")
@Config
public class TurretTest extends OpMode {

    public static double targetAngle;
    public static int flywheelSpeed;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Actuation.turretMoveTowards(targetAngle);
        Actuation.setFlywheel(flywheelSpeed);
    }
}
