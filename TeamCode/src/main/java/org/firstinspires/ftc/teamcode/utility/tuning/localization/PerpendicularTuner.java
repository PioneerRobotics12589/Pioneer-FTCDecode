package org.firstinspires.ftc.teamcode.utility.tuning.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

@Config
@TeleOp(name="Perpendicular Tuner", group="tuning")
public class PerpendicularTuner extends OpMode {
    public static double measuredDist = 0.0;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();

        Actuation.drive(0, 0, gamepad1.left_stick_x);

        Actuation.packet.put("Y pos", OttoCore.robotPose.y);
        Actuation.packet.put("diff", measuredDist / OttoCore.robotPose.y);
        Actuation.packet.put("new perpendicular multiplier", ActuationConstants.Drivetrain.perpendicularMultiplier * (measuredDist /OttoCore.robotPose.y));
        Actuation.updateTelemetry();
    }
}