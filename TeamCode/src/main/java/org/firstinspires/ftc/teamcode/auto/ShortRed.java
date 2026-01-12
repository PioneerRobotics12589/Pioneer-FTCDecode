package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="ShortRed", group="Red Auto")
public class ShortRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        Pose start = new Pose(48.6, -52, Math.toRadians(-55.6));
        Pose spike1A = new Pose(8.5, -28, Math.toRadians(-92.0));
        Pose spike1B = new Pose(8.5, -50, Math.toRadians(-94));
        Pose spike2A = new Pose(-17, -20, Math.toRadians(-94.0));
        Pose spike2B = new Pose(-17, -50, Math.toRadians(-96.0));
        Pose gatePose = new Pose(-7.4, -40, Math.toRadians(180.0));
        Pose launchPose = new Pose(12.0, -12, Math.toRadians(-40.0)); // Make farther back

        Trajectory preloads = new Trajectory(start);

        Trajectory launch = new Trajectory()
                .action(() -> Actuation.setFlywheel(1600))
                .lineTo(AutoMovement.closestShortLaunch("red"))
                .action(() -> {
                    try {
                        AutoMovement.launch();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

        Trajectory spike1 = new Trajectory()
                .lineTo(spike1A)
                .lineToIntake(spike1B, 0.6, 0.7);

        Trajectory spike2 = new Trajectory()
                .lineTo(spike2A)
                .lineToIntake(spike2B, 0.6, 0.7);

        Trajectory gate = new Trajectory()
                .lineTo(gatePose);

        waitForStart();

        preloads.run();
        launch.run();
        spike1.run();
        launch.run();
        spike2.run();
        launch.run();
        gate.run();

        telemetry.addData("X=", OttoCore.robotPose.x);
        telemetry.addData("Y=", OttoCore.robotPose.y);
        telemetry.addData("theta=", OttoCore.robotPose.heading);
        telemetry.update();
    }
}
