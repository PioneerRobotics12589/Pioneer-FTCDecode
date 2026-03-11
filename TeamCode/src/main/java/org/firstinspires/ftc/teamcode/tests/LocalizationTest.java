package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.localization.PinpointControl;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="Localization Test")
public class LocalizationTest extends OpMode {

    public static double xOffset = -4.35, yOffset = -6.0;
    private static ArrayList<Pose> tagAvg = new ArrayList<>();

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        OttoCore.setup(hardwareMap);
    }

    @Override
    public void start() {
        OttoCore.setPose(new Pose(0, 0, Math.toRadians(0)));
    } // Pose initialization must be called in start or after waitForStart()

    @Override
    public void loop() {
        OttoCore.updatePosition();
        OttoCore.displayPosition();

        // *** Tag Localization ***
        Actuation.setPipeline(ActuationConstants.LimelightConsts.PIPELINE_APRILTAG);

        LLResult result = Actuation.getLLResult();

        Pose poseLL = new Pose(0, 0, 0);
        Pose avgLL = new Pose(0, 0, 0);

        if (result != null & result.isValid()) {
            Pose3D botPose = result.getBotpose();
            int tags = result.getBotposeTagCount();

            if (tags >= 1) {
                // Calculated Pose in inches
                poseLL = new Pose(-botPose.getPosition().x * 39.37, -botPose.getPosition().y * 39.37, OttoCore.robotPose.heading);
                tagAvg.add(poseLL);
                if (tagAvg.size() > 5) {
                    tagAvg.remove(0);
                }
                for (Pose p : tagAvg) {
                    avgLL.x += p.x / tagAvg.size();
                    avgLL.y += p.y / tagAvg.size();
                    avgLL.heading = p.heading;
                }
            }
        }


        PinpointControl.setOffsets(xOffset, yOffset);
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
        Actuation.packet.put("Move",gamepad1.left_stick_y);
        Actuation.packet.put("Strafe", -gamepad1.left_stick_x);
        Actuation.packet.put("Turn", -gamepad1.right_stick_x);

        Actuation.packet.put("Pos LL", poseLL);
        Actuation.packet.put("Avg LL", avgLL);
        Actuation.packet.put("Pos Odo", OttoCore.robotPose.toString());
        Actuation.packet.put("Vel", OttoCore.getVelocity().toString());

        Actuation.updateTelemetry();
    }
}