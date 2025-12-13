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

    private static boolean prevTriangle = false, prevSquare = false;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    public void loop() {
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

        if (gamepad1.right_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);
        }
        else if (gamepad1.left_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);
        } else if (gamepad1.cross) {

            double[] vals = Actuation.launchVals("blue");
            Pose position = new Pose(OttoCore.robotPose);
            double robotAngle = vals[0];
            int flyVel = (int) vals[1];
            while (position.heading < robotAngle-0.075 || position.heading > robotAngle+0.075) {
                double[] moveValues = OttoCore.moveTowardsSignals(new Pose(position.x, position.y, robotAngle), 0.0, 0.5);
                Actuation.drive(gamepad1.left_stick_y, moveValues[1], -gamepad1.left_stick_x);
                OttoCore.updatePosition();
                position = new Pose(OttoCore.robotPose);

            }
            Actuation.setFlywheel(flyVel);
            Actuation.checkFlywheelSpeed(gamepad1, flyVel);

        } else if (gamepad1.triangle && !prevTriangle) {

            prevTriangle = gamepad1.triangle;
            while (!gamepad1.triangle || !prevTriangle) {
                prevTriangle = gamepad1.triangle;
                double turnRate = ArtifactDetection.trackArtifact("green");
                Actuation.packet.put("Pixel X", ArtifactDetection.locateArtifact("purple"));
                Actuation.drive(gamepad1.left_stick_y, turnRate, -gamepad1.left_stick_x);
            }

        } else if (gamepad1.square && !prevSquare) {

            prevSquare = gamepad1.square;
            while (!gamepad1.square || !prevSquare) {
                prevSquare = gamepad1.square;
                double turnRate = ArtifactDetection.trackArtifact("purple");
                Actuation.packet.put("Pixel X", ArtifactDetection.locateArtifact("purple"));
                Actuation.drive(gamepad1.left_stick_y, turnRate, -gamepad1.left_stick_x);
            }

        } else {

            Actuation.setFlywheel(0);
            Actuation.checkFlywheelSpeed(gamepad1, 0);

        }

        Actuation.setIntake(gamepad1.left_trigger > 0.5);
        Actuation.runBackwards(gamepad1.right_trigger > 0.5);
        Actuation.setTransfer(gamepad1.right_trigger > 0.5);
    }
}