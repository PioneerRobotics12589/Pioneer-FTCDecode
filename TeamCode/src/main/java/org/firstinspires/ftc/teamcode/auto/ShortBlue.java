package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="ShortBlue", group="blue")
public class ShortBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        Pose start = new Pose(52, 45, Math.toRadians(52.6));
        Pose spike1A = new Pose(10, 27, Math.toRadians(92.0));
        Pose spike1B = new Pose(10, 50, Math.toRadians(92.0));
        Pose spike2A = new Pose(-15, 27, Math.toRadians(94.0));
        Pose spike2B = new Pose(-15, 50, Math.toRadians(94.0));
        Pose gatePose = new Pose(0, 40.0, Math.toRadians(0.0));
        Pose launchPose = new Pose(12.0, 12, Math.toRadians(40.0)); // Make farther back

        Trajectory preloads = new Trajectory(start);

        Trajectory launch = new Trajectory()
                .action(() -> Actuation.setFlywheel(1600))
                .lineTo(new Pose(launchPose.x, launchPose.y, AutoMovement.goalRotation(launchPose, FieldConstants.Goal.blue)))
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
    }
}
