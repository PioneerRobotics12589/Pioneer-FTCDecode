package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.utility.localization.IMUControl;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Awe(sigma) Sauce Blue")
@Config
public class RobotTeleOpBlue extends OpMode {
    private int shootingSpeed;
    private ElapsedTime runtime = new ElapsedTime();
    private double time;

//    private Thread turretOp;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        AutoLaunch.setTeam("blue");
        Actuation.setBlocker(false);
    }

    public void start() {
        runtime.reset();
    }

    public void loop() {
        time = runtime.seconds();
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
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x*0.75, -gamepad1.left_stick_x*0.75);

        if (gamepad1.dpad_left) {
            // Speed up flywheel to shoot from the long launch zone
            shootingSpeed = ActuationConstants.Launcher.longLaunch;

        } else if (gamepad1.dpad_right) {
            // Speed up flywheel to shoot from the short launch zone
            shootingSpeed = ActuationConstants.Launcher.shortLaunch;
        }
        /*if (gamepad1.right_bumper) {
            Actuation.intake.setPower(-1.0);
        }*/
        if (gamepad1.left_stick_button) {
            OttoCore.setPose(FieldConstants.Reset.blueCorner);
        } else if (gamepad1.right_stick_button) {
            OttoCore.setPose(new Pose(0, 0, 0));
        }

        if (gamepad1.left_trigger > 0.5) {
            // Shooting Mode
            Actuation.shoot(true);
        } else if (gamepad1.right_trigger > 0.5) {
            Actuation.reverse(true);
        } else if (gamepad1.right_bumper) {
            // Intake Mode
            Actuation.runIntake(true);
            Actuation.runTransfer(true);
        } else {
            // Everything Off
            Actuation.runIntake(false);
            Actuation.runTransfer(false);
        }

        Actuation.setBlocker(false);

//        Actuation.setFlywheel(shootingSpeed);
        Actuation.checkFlywheelSpeed(gamepad1, shootingSpeed);
        //Actuation.shoot(gamepad1.left_trigger > 0.5);
        //Actuation.runIntake(gamepad1.right_trigger > 0.5);
        //Actuation.runTransfer(gamepad1.right_bumper);
        //Actuation.intake(gamepad1.right_bumper);
        //Actuation.runIntake(gamepad1.right_bumper);
        //Actuation.reverse(gamepad1.right_trigger > 0.5);
        Actuation.setLaunchIndicator(time);
        AutoMovement.turretOperation("blue");

        FieldConstants.Goal.blue = new Point(FieldConstants.Goal.blueX, FieldConstants.Goal.blueY);

        OttoCore.updatePosition();
        OttoCore.displayPosition();
        telemetry.addData("Turret Pos", Math.toDegrees(Actuation.getTurretGlobal()));
        telemetry.update();


    }
}