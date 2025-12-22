package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.PIDController;
import org.firstinspires.ftc.teamcode.utility.cameraVision.ArtifactDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name = "Awe(sigma) Sauce")
@Config
public class RobotTeleOp extends OpMode {
    private boolean trackPurple = false, trackGreen = false;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    public void loop() {

        boolean triangle = gamepad1.triangleWasPressed();
        boolean square = gamepad1.squareWasPressed();

        // Toggle tracking
        if (triangle) {
            trackGreen = !trackGreen;
            trackPurple = false;
        } else if (square) {
            trackGreen = false;
            trackPurple = !trackPurple;
        }

        if (trackPurple) {
            // Track purple artifacts (while moving)
            double turnRate = ArtifactDetection.trackArtifact("purple");
            telemetry.addData("Pixel X", ArtifactDetection.trackArtifact("purple"));
            telemetry.update();
            gamepad1.setLedColor(255, 0, 255, 100);
            Actuation.drive(gamepad1.left_stick_y, turnRate, -gamepad1.left_stick_x);

        } else if (trackGreen) {
            // Track green artifacts (while moving)
            double turnRate = ArtifactDetection.trackArtifact("green");
            telemetry.addData("Pixel X", ArtifactDetection.trackArtifact("green"));
            telemetry.update();
            gamepad1.setLedColor(0, 255, 0, 100);
            Actuation.drive(gamepad1.left_stick_y, turnRate, -gamepad1.left_stick_x);

        } else {
            if (gamepad1.right_bumper) {
                // Speed up flywheel to shoot from the long launch zone
                Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);

            } else if (gamepad1.left_bumper) {
                // Speed up flywheel to shoot from the short launch zone
                Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);

            } else if (gamepad1.cross) {
                // Turn towards the goal and determines the correct launch velocity to shoot into the goal

                double[] vals = Actuation.launchVals("blue");
                Pose position = new Pose(OttoCore.robotPose);


                int flyVel = (int) vals[1];

                Actuation.setFlywheel(flyVel);
                Actuation.checkFlywheelSpeed(gamepad1, flyVel);
            } else {
                Actuation.setFlywheel(0);
                Actuation.checkFlywheelSpeed(gamepad1, 0);
            }

            Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            Actuation.runIntake(gamepad1.right_trigger > 0.5);
            Actuation.runTransfer(gamepad1.right_trigger > 0.5);
            Actuation.reverse(gamepad1.left_trigger > 0.5);
        }
    }
}