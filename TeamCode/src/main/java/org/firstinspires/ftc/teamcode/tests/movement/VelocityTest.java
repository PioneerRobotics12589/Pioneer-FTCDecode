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

    PIDController vertical, lateral, heading;
    public static PIDCoeffs lateralGains = new PIDCoeffs(0.2, 45, 0.08);
    public static PIDCoeffs verticalGains = new PIDCoeffs(0.25, 40, 0.09);
    public static PIDCoeffs headingGains = new PIDCoeffs(4.0, 30, 0.2);

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double headingTarget = 0;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

        vertical = new PIDController(verticalGains);
        heading = new PIDController(headingGains);
        lateral = new PIDController(lateralGains);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();
        vertical.updateCoeffs(verticalGains);
        heading.updateCoeffs(headingGains);
        lateral.updateCoeffs(lateralGains);

        double xSignal = vertical.calculateSignal(xTarget, OttoCore.robotPose.x);
        double ySignal = lateral.calculateSignal(yTarget, OttoCore.robotPose.y);
        double headingSignal = heading.calculateSignal(headingTarget, OttoCore.robotPose.heading);

        double clampX = Math.max(-1.0, Math.min(1, xSignal));
        double clampY = Math.max(-1.0, Math.min(1, ySignal));
        double clampHeading = -Math.max(-1.0, Math.min(1, headingSignal));

        double move = clampX * Math.cos(OttoCore.robotPose.heading) + clampY * Math.sin(OttoCore.robotPose.heading);
        double strafe = clampX * Math.sin(OttoCore.robotPose.heading) - clampY * Math.cos(OttoCore.robotPose.heading);

        Actuation.drive(move * 0.8, clampHeading * 0.8, strafe * 0.8);

        Actuation.packet.put("xSignal", xSignal);
        Actuation.packet.put("ySignal", ySignal);
        Actuation.packet.put("hSignal", headingSignal);

        Actuation.updateTelemetry();
    }
}
