package org.firstinspires.ftc.teamcode.tests.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDCoeffs;

@TeleOp(name= "VelocityTest")
@Config
public class VelocityTest extends OpMode {
    FtcDashboard dashboard;

    PIDController lateral, heading;
    public static PIDCoeffs lateralGains = new PIDCoeffs(0.25, 0.0, 0.03);
    public static PIDCoeffs headingGains = new PIDCoeffs(4.0, 0.0, 0.1);

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double headingTarget = 0;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        lateral = new PIDController(lateralGains);
        heading = new PIDController(headingGains);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();
        lateral.updateCoeffs(lateralGains);
        heading.updateCoeffs(headingGains);

        double xSignal = lateral.calculateSignal(xTarget, OttoCore.robotPose.x);
        double ySignal = lateral.calculateSignal(yTarget, OttoCore.robotPose.y);
        double headingSignal = heading.calculateSignal(headingTarget, OttoCore.robotPose.heading);

        double clampX = Math.max(-1.0, Math.min(1, xSignal));
        double clampY = Math.max(-1.0, Math.min(1, ySignal));
        double clampHeading = -Math.max(-1.0, Math.min(1, headingSignal));

        double move = clampX * Math.cos(OttoCore.robotPose.heading) + clampY * Math.sin(OttoCore.robotPose.heading);
        double strafe = clampX * Math.sin(OttoCore.robotPose.heading) - clampY * Math.cos(OttoCore.robotPose.heading);

        Actuation.drive(move * 0.8, clampHeading * 0.8, strafe * 0.8);

        Actuation.packet.put("x pos", OttoCore.robotPose.x);
        Actuation.packet.put("heading", OttoCore.robotPose.heading);
        Actuation.packet.put("target heading", headingTarget);
        Actuation.packet.put("heading signal", headingSignal);

        Actuation.updateTelemetry();
    }
}
