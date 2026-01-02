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

@Autonomous(name="LongBlue", group = "Blue Auto")
@Config
public class LongBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);

        Pose start = new Pose(-64.77, -10.62, Math.toRadians(-4.4));
        Pose spike3A = new Pose(-35.75, -28, Math.toRadians(-88.7));
        Pose spike3B = new Pose(-35.75, -45, Math.toRadians(-88.7));
        Pose spike2A = new Pose(-15.5, -28, Math.toRadians(-94.0));
        Pose spike2B = new Pose(-15.5, -45, Math.toRadians(-94.0));
        Pose launchPose = new Pose(-57, -9.5, Math.toRadians(-20));
        Pose gatePose = new Pose(-7.4, -49, Math.toRadians(-92));

        // Start
        Trajectory preloads = new Trajectory(start);

        Trajectory spike3 = new Trajectory()
                .lineTo(spike3A)
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true, false, ActuationConstants.Intake.transferSpeed*0.5))
                .lineTo(spike3B, 0.4, 0.5)
                .action(() -> Actuation.runTransfer(false, false))
                .action(() -> Actuation.runIntake(false));

        Trajectory spike2 = new Trajectory()
                .lineTo(spike2A)
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true, false, ActuationConstants.Intake.transferSpeed*0.3))
                .lineTo(spike2B, 0.4, 0.5)
                .action(() -> Actuation.runTransfer(false, false))
                .action(() -> Actuation.runIntake(false));

//        Trajectory pickup_dump = new Trajectory()
//                .lineToMobile(FieldConstants.Dump.red)
//                .action(() -> AutoMovement.alignToArtifactAuto("purple"))
//                .action(() -> Actuation.runIntake(true))
//                .action(() -> Actuation.runTransfer(true, false))
//                .action(() -> AutoMovement.moveForward(20))
//                .action(() -> Actuation.runIntake(false))
//                .action(() -> Actuation.runTransfer(false, false));

        Trajectory launch = new Trajectory()
                .action(() -> AutoMovement.autoFlywheelVel(launchPose, FieldConstants.Goal.blue))
                .lineTo(new Pose(launchPose.x, launchPose.y, AutoMovement.goalRotation(launchPose, FieldConstants.Goal.blue)))
                .action(() -> {
                    try {
                        AutoMovement.launch();
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
        launch.run();
        spike3.run();
        launch.run();
        spike2.run();
        gate.run();
//        launch.run();
//        pickup_dump.run();
//        launch.run();
//        pickup_dump.run();
//        launch.run();
    }
}
