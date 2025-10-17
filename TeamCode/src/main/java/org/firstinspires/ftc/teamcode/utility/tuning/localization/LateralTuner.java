package org.firstinspires.ftc.teamcode.utility.tuning.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

@Config
@TeleOp(name="Lateral Tuner", group="tuning")
public class LateralTuner extends OpMode {
    public static double measuredAngle = 180;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();

        Actuation.packet.put("Heading", Math.toDegrees(OttoCore.robotPose.heading));
        Actuation.packet.put("diff", Math.toRadians(measuredAngle) / OttoCore.robotPose.heading);
        Actuation.packet.put("new lateral multiplier", ActuationConstants.Drivetrain.lateralMultiplier / (Math.toRadians(measuredAngle)/OttoCore.robotPose.heading));
        Actuation.updateTelemetry();
    }
}