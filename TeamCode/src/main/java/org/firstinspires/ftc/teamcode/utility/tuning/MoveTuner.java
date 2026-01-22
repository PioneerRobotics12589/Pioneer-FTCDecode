package org.firstinspires.ftc.teamcode.utility.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

@Config
@TeleOp(name="Move Tuner", group="tuning")
public class MoveTuner extends OpMode {
    public static double measuredDistance = 20.0;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();
        Actuation.packet.addLine("Strafe the robot to the left");
        Actuation.packet.put("Distance", OttoCore.robotPose.y);

        double multiplier = OttoCore.robotPose.y / measuredDistance;
        Actuation.packet.put("New Forward Offset", ActuationConstants.Drivetrain.forward_offset * multiplier);

        Actuation.updateTelemetry();
    }
}
