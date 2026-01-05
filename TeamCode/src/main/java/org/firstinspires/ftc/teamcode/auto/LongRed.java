package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="LongRed", group = "Red Auto")
@Config
public class LongRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);

        Pose start = new Pose(-64.5, -14, Math.toRadians(0.0));
        Pose spike3A = new Pose(-34.5, -23, Math.toRadians(-90)); // Needs Changing
        Pose spike3B = new Pose(-34.5, -52.5, Math.toRadians(-90)); // Needs Changing
        Pose spike2A = new Pose(-11, -15, Math.toRadians(-85.0)); // Needs Changing
        Pose spike2B = new Pose(-11, -45, Math.toRadians(-85.0)); // Needs Changing
        Pose launchPose1 = new Pose(-58.7, -7.14, Math.toRadians(-21.5)); // Needs Changing
        Pose launchPose2 = new Pose(-58.7, -7.14, Math.toRadians(-16.5)); // Needs Changing
        Pose launchPose3 = new Pose(-58.7, -7.14, Math.toRadians(-14)); // Needs Changing
        Pose gatePose = new Pose(3, -30, Math.toRadians(-85)); // Needs Changing
        Pose endPose = new Pose(-58.7, -35, Math.toRadians(-85)); // Needs Changing

        // Start
        Trajectory preloads = new Trajectory(start);

        Trajectory spike3 = new Trajectory()
                .lineTo(spike3A)
                .lineToIntake(spike3B, 0.7, 0.7);
        Trajectory spike3end = new Trajectory()
                .lineTo(spike3A);
//                .lineToIntake(spike3B, 0.67, 0.4);
        Trajectory end = new Trajectory()
                .lineTo(endPose);
//                .lineToIntake(spike3B, 0.67, 0.4);

        Trajectory spike2 = new Trajectory()
                .lineTo(spike2A)
                .lineToIntake(spike2B, 0.7, 0.7);

//        Trajectory pickup_dump = new Trajectory()
//                .lineToMobile(FieldConstants.Dump.red)
//                .action(() -> AutoMovement.alignToArtifactAuto("purple"))
//                .action(() -> Actuation.runIntake(true))
//                .action(() -> Actuation.runTransfer(true, false))
//                .action(() -> AutoMovement.moveForward(20))
//                .action(() -> Actuation.runIntake(false))
//                .action(() -> Actuation.runTransfer(false, false));

        Trajectory launch1 = new Trajectory()
                .action(() -> Actuation.setFlywheel(1780))
                .lineTo(launchPose1)
                .action(() -> {
                    try {
                        AutoMovement.launch(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });
        Trajectory launch2 = new Trajectory()
                .action(() -> Actuation.setFlywheel(1800))
                .lineTo(launchPose2)
                .action(() -> {
                    try {
                        AutoMovement.launch(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });
        Trajectory launch3 = new Trajectory()
                .action(() -> Actuation.setFlywheel(1790))
                .lineTo(launchPose3)
                .action(() -> {
                    try {
                        AutoMovement.launch(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

//        Trajectory lever = new Trajectory()
//                .lineTo(FieldConstants.Gate.red)
//                .action(() -> AutoMovement.moveForward(2.5))
//                .action(() -> sleep(2500))
//                .action(() -> AutoMovement.moveForward(-2.5));
        Trajectory gate = new Trajectory()
                .lineTo(gatePose);


        waitForStart();

        // FULL AUTO (18 artifacts? 6 teammate, 12 us)
        preloads.run();
        launch1.run();
//        spike3.run();
//        launch2.run();
//        spike2.run();
//        launch3.run();
        end.run();
//        gate.run();
//        spike3end.run();
//        launch.run();
//        pickup_dump.run();
//        launch.run();
//        pickup_dump.run();
//        launch.run();
    }
}
