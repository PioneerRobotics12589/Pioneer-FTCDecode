package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.PIDController;
import org.firstinspires.ftc.teamcode.utility.cameraVision.ArtifactDetection;

@TeleOp(name="Artifact Tracking Test")
@Config
public class ArtifactTrackingTest extends OpMode {

    public static double kp, ki, kd;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        ActuationConstants.LimelightConsts.head_PID = new PIDController(kp, ki, kd);
        double turnRate = ArtifactDetection.trackArtifact("purple");
        Actuation.drive(0.0, turnRate, 0.0);
        Actuation.packet.addLine("Pixel X= " + ArtifactDetection.locateArtifact("purple"));
        Actuation.updateTelemetry();
    }
}