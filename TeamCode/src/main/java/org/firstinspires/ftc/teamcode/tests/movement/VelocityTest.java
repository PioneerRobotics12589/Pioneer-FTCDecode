package org.firstinspires.ftc.teamcode.tests.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDCoeffs;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Disabled
@TeleOp(name= "VelocityTest")
@Config
public class VelocityTest extends OpMode {
    FtcDashboard dashboard;

    PIDController vertical, lateral, rotational;
    public static PIDCoeffs lateralGains = new PIDCoeffs(0.2, 50, 0.08);
    public static PIDCoeffs verticalGains = new PIDCoeffs(0.25, 40, 0.09);
    public static PIDCoeffs rotationalGains = new PIDCoeffs(4.0, 30, 0.2);

    public static Pose targetPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        OttoCore.robotPose = new Pose(0, 0, 0);

        vertical = new PIDController(verticalGains);
        rotational = new PIDController(rotationalGains);
        lateral = new PIDController(lateralGains);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();
        lateral.updateCoeffs(lateralGains);
        vertical.updateCoeffs(verticalGains);
        rotational.updateCoeffs(rotationalGains);

        double vertSignal = vertical.calculateSignal(targetPose.x, OttoCore.robotPose.x);
        double latSignal = lateral.calculateSignal(targetPose.y, OttoCore.robotPose.y);
        double rotSignal = rotational.calculateSignal(targetPose.heading, OttoCore.robotPose.heading);

        double clampVert = -Math.max(-1.0, Math.min(1, vertSignal));
        double clampLat = -Math.max(-1.0, Math.min(1, latSignal));
        double clampRot = Math.max(-1.0, Math.min(1, rotSignal));

        double move = clampVert * Math.cos(OttoCore.robotPose.heading) + clampLat * Math.sin(OttoCore.robotPose.heading);
        double strafe = clampVert * Math.sin(OttoCore.robotPose.heading) - clampLat * Math.cos(OttoCore.robotPose.heading);

        Actuation.drive(move * 0.8, clampRot * 0.8, strafe * 0.8);

        Actuation.packet.put("xSignal", vertSignal);
        Actuation.packet.put("ySignal", latSignal);
        Actuation.packet.put("hSignal", rotSignal);
        Actuation.packet.put("Target X", targetPose.x);
        Actuation.packet.put("Target Y", targetPose.y);
        Actuation.packet.put("Target H", targetPose.heading);
        Actuation.packet.put("X", OttoCore.robotPose.x);
        Actuation.packet.put("Y", OttoCore.robotPose.y);
        Actuation.packet.put("H", OttoCore.robotPose.heading);

        Actuation.updateTelemetry();
    }
}