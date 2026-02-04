package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Awe(sigma) Sauce Blue")
@Config
public class RobotTeleOpBlue extends OpMode {
    private boolean trackPurple = false;
    private boolean trackGreen = false;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        AutoLaunch.setTeam("blue");
    }

    public void loop() {
        telemetry.addLine("X=" + OttoCore.robotPose.x + " Y=" + OttoCore.robotPose.y + "θ=" + Math.toDegrees(OttoCore.robotPose.heading));

        boolean autoLaunch = gamepad2.cross;
        boolean autoLaunch1 = gamepad2.circle;

        // Toggles
        if (gamepad1.squareWasPressed()) {
            trackPurple = !trackPurple;
            if (trackPurple) {
                gamepad1.setLedColor(255, 0, 255, 3000);
                trackGreen = false;
            }
        } else if (gamepad1.triangleWasPressed()) {
            trackGreen = !trackGreen;
            if (trackGreen) {
                gamepad1.setLedColor(0, 255, 0, 3000);
                trackPurple = false;
            }
        }


        if (trackPurple) {
            // Track purple artifacts (while moving)
            AutoMovement.alignToArtifact("purple", gamepad1.left_stick_y, -gamepad1.right_stick_x*0.75);
            telemetry.addLine("Tracking Purple");

        } else if (trackGreen) {
            // Track green artifacts (while moving)
            AutoMovement.alignToArtifact("green", gamepad1.left_stick_y, -gamepad1.right_stick_x*0.75);
            telemetry.addLine("Tracking Green");

        } else if (autoLaunch) {
            // Auto launch artifacts (while stationary)
            gamepad1.setLedColor(255, 255, 0, 3000);
            telemetry.addLine("Tracking Goal");
            AutoLaunch.updateAutoLaunchS(OttoCore.robotPose);
            AutoLaunch.rotate();
            AutoLaunch.setFlywheel();

        } else if (autoLaunch1) {
            // Auto launch artifacts (while moving)
            gamepad1.setLedColor(255, 255, 0, 3000);
            telemetry.addLine("Tracking Goal");
            AutoLaunch.updateAutoLaunchM(OttoCore.robotPose);
            AutoLaunch.rotate(gamepad2.left_stick_y, -gamepad2.left_stick_x);
            AutoLaunch.setFlywheel();

        } else if (gamepad2.dpadDownWasPressed()){
            // Go to park zone
            Trajectory park = new Trajectory()
                    .lineToTeleOp(FieldConstants.Park.blue, () -> gamepad2.dpadDownWasPressed());
            park.run();

        } else {
            Actuation.drive(gamepad2.left_stick_y, -gamepad2.right_stick_x, -gamepad2.left_stick_x*0.75);
        }

        if (gamepad2.right_bumper) {
            // Speed up flywheel to shoot from the long launch zone
            Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
            Actuation.checkFlywheelSpeed(gamepad2, ActuationConstants.Launcher.longLaunch);

        } else if (gamepad2.left_bumper) {
            // Speed up flywheel to shoot from the short launch zone
            Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
            Actuation.checkFlywheelSpeed(gamepad2, ActuationConstants.Launcher.shortLaunch);

        } else if (!autoLaunch1 && !autoLaunch){
            Actuation.setFlywheel(0);
            Actuation.checkFlywheelSpeed(gamepad2, 0);
        }

       // Actuation.shoot(gamepad1.right_trigger > 0.5, gamepad2.right_trigger > 0.5);
        Actuation.runIntake(gamepad1.right_trigger > 0.5, gamepad2.right_trigger > 0.5);
        Actuation.runTransfer(gamepad2.right_trigger > 0.5);
        Actuation.reverse(gamepad2.left_trigger > 0.5);
        if (gamepad2.dpad_left) {
            Actuation.controlTurret(40, 1);
        } else if (gamepad2.dpad_right) {
            Actuation.controlTurret(-40, 1);
        }
        AutoMovement.turretOperation("blue");
//        Actuation.setLaunchIndicator();
        OttoCore.updatePosition();
        telemetry.update();
    }
}