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
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

@TeleOp(name = "Awe(sigma) Sauce Red")
@Config
public class RobotTeleOpRed extends OpMode {
    private int shootingSpeed;

//    private Thread turretOp;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        AutoLaunch.setTeam("blue");
    }
    /*public void start() {
utoMovement.turretOperation("blue", gamepad1)
        turretOp.start();
    }*/

    public void loop() {
        telemetry.addLine("X=" + OttoCore.robotPose.x + "\nY=" + OttoCore.robotPose.y + "\nθ=" + Math.toDegrees(OttoCore.robotPose.heading));

      /*  // Toggles
        if (gamepad2.squareWasPressed()) {
            trackArtifact = !trackArtifact;
            if (trackArtifact) {
                gamepad2.setLedColor(255, 0, 255, 3000);
            }
        }


        if (trackArtifact) {
            // Track purple artifacts (while moving)
            AutoMovement.autoIntakeArtifact();
            telemetry.addLine("Tracking Purple");

        } else if (gamepad2.dpadDownWasPressed()){
            // Go to park zone
            //Trajectory park = new Trajectory().lineToTeleOp(FieldConstants.Park.blue, () -> gamepad2.dpadDownWasPressed());
            //park.run();

        } else {
            Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x*0.75);
        }*/
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x*0.75);

        if (gamepad1.dpad_left) {
            // Speed up flywheel to shoot from the long launch zone
            shootingSpeed = ActuationConstants.Launcher.longLaunch;

        } else if (gamepad1.dpad_right) {
            // Speed up flywheel to shoot from the short launch zone
            shootingSpeed = ActuationConstants.Launcher.shortLaunch;
        }
        if (gamepad1.right_bumper) {
            Actuation.intake.setPower(-1.0);
        }
        else {
            Actuation.intake.setPower(0.0);
        }
        if (gamepad2.dpad_left) {
            Actuation.turretMoveTowards(Math.toRadians(-50));
            shootingSpeed = ActuationConstants.Launcher.shortLaunch;
        }
        else if (gamepad2.dpad_up) {
            Actuation.turretMoveTowards(Math.toRadians(0));
        }
        else if (gamepad2.dpad_down) {
            Actuation.turretMoveTowards(Math.toRadians(-30));
            shootingSpeed = ActuationConstants.Launcher.longLaunch;
        }
        Actuation.setFlywheel(shootingSpeed);
        Actuation.checkFlywheelSpeed(gamepad1, shootingSpeed);
        Actuation.shoot(gamepad1.left_trigger > 0.5);
        //Actuation.runIntake(gamepad1.right_trigger > 0.5);
        Actuation.runTransfer(gamepad1.right_bumper);
        //Actuation.runIntake(gamepad1.right_bumper);
        Actuation.reverse(gamepad1.right_trigger > 0.5);
        // Actuation.setLaunchIndicator();
//        if (gamepad1.dpad_left) {
//            Actuation.controlTurret(1.0);
//        } else if (gamepad1.dpad_right) {
//            Actuation.controlTurret(-1.0);
//        }
        OttoCore.updatePosition();
        telemetry.addData("Turret Pos", Math.toDegrees(Actuation.getTurretLocal()));
        telemetry.update();
    }
}