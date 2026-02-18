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
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Awe(sigma) Sauce Blue")
@Config
public class RobotTeleOpBlue extends OpMode {
    private int shootingSpeed;
    private boolean trackArtifact = false;
    private Thread turretOp;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        AutoLaunch.setTeam("blue");
    }

    public void start() {
        turretOp = AutoMovement.turretOperation("blue");
        turretOp.start();
    }

    public void loop() {
        telemetry.addLine("X=" + OttoCore.robotPose.x + "\nY=" + OttoCore.robotPose.y + "\nθ=" + Math.toDegrees(OttoCore.robotPose.heading));

        // Toggles
        if (gamepad1.squareWasPressed()) {
            trackArtifact = !trackArtifact;
            if (trackArtifact) {
                gamepad1.setLedColor(255, 0, 255, 3000);
            }
        } else if (gamepad2.crossWasPressed()) {
            AutoMovement.toggleTracking();
        } else if (gamepad1.dpad_down) {
            OttoCore.robotPose = new Pose(0, 0, 0);
        }


       /* if (trackArtifact) {
            // Track purple artifacts (while moving)
            AutoMovement.autoIntakeArtifact();
            telemetry.addLine("Tracking Purple");

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

        }*/

        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x*0.75);

        if (gamepad1.dpad_left) {
            // Speed up flywheel to shoot from the long launch zone
            shootingSpeed = ActuationConstants.Launcher.longLaunch;

        } else if (gamepad1.dpad_right) {
            // Speed up flywheel to shoot from the short launch zone
            shootingSpeed = ActuationConstants.Launcher.shortLaunch;
        }
//        Actuation.setFlywheel(shootingSpeed);
//        Actuation.checkFlywheelSpeed(gamepad1, shootingSpeed);
        Actuation.shoot(gamepad1.left_trigger > 0.5);
        //Actuation.runIntake(gamepad1.right_trigger > 0.5);
        Actuation.runTransfer(gamepad1.right_bumper);
        Actuation.runIntake(gamepad1.right_bumper);
        Actuation.reverse(gamepad1.right_trigger > 0.5);
//        Actuation.setLaunchIndicator();
        OttoCore.updatePosition();
        telemetry.update();
    }

    public void stop() {
        turretOp.interrupt();
        try {
            turretOp.join(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}