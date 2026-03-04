package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

//@Disabled
@Autonomous(name="Autonomous")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();

//        OttoCore.robotPose = new Pose(0, 0, 0);
        OttoCore.setPose(new Pose(-10, 0, Math.toRadians(90))); // < DOES NOT WORK
//        OttoCore.setPose(new Pose(0, 0, 0)); // < WORKS PERFECTLY FINE

        Trajectory movement = new Trajectory()
//                .lineTo(new Pose(0, -30, Math.toRadians(0)), 1.0, 1.0);
//                .lineTo(new Pose(30, 0, Math.toRadians(-90)), 1.0, 1.0)
//                .lineTo(new Pose(30, -30, Math.toRadians(180)), 1.0, 1.0)
//                .lineTo(new Pose(0, -30, Math.toRadians(90)), 1.0, 1.0)
//                .lineTo(new Pose(0, 0, Math.toRadians(0)), 1.0, 1.0);
        .lineTo(new Pose(30, 0, Math.toRadians(-90)), 1.0, 1.0);
//        .lineTo(new Pose(30, -20, Math.toRadians(-90)), 1.0, 1.0);

        movement.run();
    }
}