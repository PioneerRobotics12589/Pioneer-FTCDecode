package org.firstinspires.ftc.teamcode.utility.tuning.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

@Config
@TeleOp(name="Center Tuner", group="tuning")
public class CenterTuner extends OpMode {
    public static double measuredDist = 0.0;
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();

        Actuation.drive(gamepad1.left_stick_y, 0.0, 0.0);

        Actuation.packet.put("X pos", OttoCore.robotPose.x);
        Actuation.packet.put("diff", measuredDist / OttoCore.robotPose.x);
        Actuation.packet.put("new center multiplier: ", ActuationConstants.Drivetrain.centerMultiplier * (measuredDist /OttoCore.robotPose.x));
        Actuation.updateTelemetry();
    }
}