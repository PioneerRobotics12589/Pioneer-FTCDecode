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
    public static double llxM = 1.0, llyM = 1.0, llxB = 0.0, llyB = 0.0, llF = -0.263;

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

        Actuation.limelight.updateRobotOrientation(Math.toDegrees(OttoCore.robotPose.heading));
        LLResult result = Actuation.getLLResult();

        Pose poseLL = new Pose(0, 0, 0);

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();
            int tags = result.getBotposeTagCount();
            double dist = result.getBotposeAvgDist();

            if (tags >= 1) {
                // Calculated Pose in inches
                poseLL = new Pose(-botPose.getPosition().x * llxM * 39.37 + llxB, -botPose.getPosition().y * 39.37 * llyM + llyB, botPose.getOrientation().getYaw(AngleUnit.RADIANS));
                poseLL = OttoCore.relativeTransform(poseLL, llF * 39.37, 0, poseLL.heading);
            }
            Actuation.packet.put("Pos LL Average Dist", dist);
        }


        PinpointControl.setOffsets(xOffset, yOffset);
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
        Actuation.packet.put("Move",gamepad1.left_stick_y);
        Actuation.packet.put("Strafe", -gamepad1.left_stick_x);
        Actuation.packet.put("Turn", -gamepad1.right_stick_x);

        Actuation.packet.put("Pos LL", poseLL);
//        Actuation.packet.put("Avg LL", avgLL);
        Actuation.packet.put("Pos Odo", OttoCore.robotPose.toString());
        Actuation.packet.put("Vel", OttoCore.getVelocity().toString());

        Actuation.updateTelemetry();
    }
}