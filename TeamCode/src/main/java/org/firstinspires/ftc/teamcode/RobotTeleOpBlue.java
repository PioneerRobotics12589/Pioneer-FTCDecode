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

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

@TeleOp(name = "Awe(sigma) Sauce Blue")
@Config
public class RobotTeleOpBlue extends OpMode {
    private int shootingSpeed;
    private ElapsedTime runtime = new ElapsedTime();
    private double time;
    private double transferSpeed;

//    private Thread turretOp;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        AutoLaunch.setTeam("blue");
    }

    public void start() {
        runtime.reset();
    }

    public void loop() {
        time = runtime.seconds();
        telemetry.addLine("X=" + OttoCore.robotPose.x + "\nY=" + OttoCore.robotPose.y + "\nθ=" + Math.toDegrees(OttoCore.robotPose.heading));

        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x*0.75, -gamepad1.left_stick_x);

        if (gamepad2.dpad_up) {
            // Speed up flywheel to shoot from the long launch zone
            shootingSpeed = ActuationConstants.Launcher.longLaunch;
            transferSpeed = -0.7;
            FieldConstants.Goal.blue = FieldConstants.Goal.blueLong;

        } else if (gamepad2.dpad_down) {
            // Speed up flywheel to shoot from the short launch zone
            shootingSpeed = ActuationConstants.Launcher.shortLaunch;
            transferSpeed = -1.0;
            FieldConstants.Goal.blue = FieldConstants.Goal.blueShort;
        }

        if (gamepad1.left_stick_button) {
            OttoCore.setPose(FieldConstants.Reset.redCorner);
        } else if (gamepad1.right_stick_button) {
            OttoCore.setPose(new Pose(0, 0, 0));
        }
        if (gamepad1.left_trigger > 0.5) {
            if (Actuation.blockerAtPos(ActuationConstants.Intake.blockerDown)) {
                Actuation.shoot(true);
            }
            Actuation.setBlocker(true);
        } else if (gamepad1.right_trigger > 0.5) {
            if (Actuation.blockerAtPos(ActuationConstants.Intake.blockerDown)) {
                //Actuation.reverse(true);
                Actuation.runIntake(true);
            }
            Actuation.setBlocker(true);
        } else if (gamepad1.right_bumper) {
            // Intake Mode
            if (Actuation.blockerAtPos(ActuationConstants.Intake.blockerUp)) {
                Actuation.runIntake(true);
                Actuation.transfer.setPower(transferSpeed);
            }
            Actuation.setBlocker(false);
        }
         else {
            // Everything Off
            Actuation.runIntake(false);
            Actuation.runTransfer(false);
        }

        Actuation.checkFlywheelSpeed(gamepad1, shootingSpeed);
        Actuation.setFlywheel(shootingSpeed);
        Actuation.setLaunchIndicator(time);
        AutoMovement.turretOperation("blue");

//        FieldConstants.Goal.blue = new Point(FieldConstants.Goal.blueX, FieldConstants.Goal.blueY);

        OttoCore.updatePosition();
        OttoCore.displayPosition();
//        telemetry.addData("Turret Pos", Math.toDegrees(Actuation.getTurretGlobal()));
        telemetry.update();


    }
}