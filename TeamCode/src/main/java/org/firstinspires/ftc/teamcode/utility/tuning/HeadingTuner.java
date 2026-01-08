package org.firstinspires.ftc.teamcode.utility.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

@Config
@TeleOp(name="Heading Tuner", group="tuning")
public class HeadingTuner extends OpMode {
    public static double measuredHeading = 180.0;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();
        Actuation.packet.put("Heading", Math.toDegrees(OttoCore.robotPose.heading));

        double multiplier = OttoCore.robotPose.heading / Math.toRadians(measuredHeading);
        Actuation.packet.put("New Track Width", ActuationConstants.Drivetrain.track_width * multiplier);

        Actuation.updateTelemetry();
    }
}
