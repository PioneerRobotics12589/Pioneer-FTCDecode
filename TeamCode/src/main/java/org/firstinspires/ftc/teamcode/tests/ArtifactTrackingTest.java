package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;
import org.firstinspires.ftc.teamcode.utility.cameraVision.ArtifactDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Disabled
@TeleOp(name="Artifact Tracking Test")
@Config
public class ArtifactTrackingTest extends OpMode {

    public static double kp, ki, kd;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        Actuation.setupLimelight(0);
    }

    @Override
    public void loop() {
        Pose artifact = ArtifactDetection.locateArtifact();
        ArtifactDetection.goToArtifact();
        Actuation.packet.put("Artifact X", artifact.x);
        Actuation.packet.put("Artifact Y", artifact.y);
        Actuation.updateTelemetry();
    }
}