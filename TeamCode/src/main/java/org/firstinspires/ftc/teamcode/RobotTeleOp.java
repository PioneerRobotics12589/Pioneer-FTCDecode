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

        if (triangle) {
            trackGreen = !trackGreen;
            trackPurple = false;
        } else if (square) {
            trackGreen = false;
            trackPurple = !trackPurple;
        }

        if (trackPurple) {

            double turnRate = ArtifactDetection.trackArtifact("purple");
            telemetry.addData("Pixel X", ArtifactDetection.trackArtifact("purple"));
            telemetry.update();
            gamepad1.setLedColor(255, 0, 255, 100);
            Actuation.drive(gamepad1.left_stick_y, turnRate, -gamepad1.left_stick_x);

        } else if (trackGreen) {

            double turnRate = ArtifactDetection.trackArtifact("green");
            telemetry.addData("Pixel X", ArtifactDetection.trackArtifact("green"));
            telemetry.update();
            gamepad1.setLedColor(0, 255, 0, 100);
            Actuation.drive(gamepad1.left_stick_y, turnRate, -gamepad1.left_stick_x);

        } else {
            if (gamepad1.right_bumper) {

                Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);

            } else if (gamepad1.left_bumper) {

                Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);

            } else if (gamepad1.cross) {
                double[] vals = Actuation.launchVals("blue");
                Pose position = new Pose(OttoCore.robotPose);
                double robotAngle = vals[0];
                int flyVel = (int) vals[1];

                while (position.heading < robotAngle - Math.PI/72 || position.heading > robotAngle + Math.PI/72) {
                    double[] moveValues = OttoCore.moveTowardsSignals(new Pose(position.x, position.y, robotAngle), 0.0, 0.5);
                    Actuation.drive(gamepad1.left_stick_y, moveValues[1], -gamepad1.left_stick_x);
                    OttoCore.updatePosition();
                    position = new Pose(OttoCore.robotPose);
                }

                Actuation.setFlywheel(flyVel);
                Actuation.checkFlywheelSpeed(gamepad1, flyVel);
            } else {
                Actuation.setFlywheel(0);
                Actuation.checkFlywheelSpeed(gamepad1, 0);
            }
            Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            Actuation.setIntake(gamepad1.left_trigger > 0.5);
            Actuation.runBackwards(gamepad1.right_trigger > 0.5);
            Actuation.setTransfer(gamepad1.right_trigger > 0.5);
        }
    }
}