package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

//@Disabled
@Autonomous(name="Autonomous")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        OttoCore.robotPose = new Pose(0, 0, 0);

        Trajectory movement = new Trajectory(new Pose(0, 0, Math.toRadians(0.0)))
                .lineTo(new Pose(30, 0, Math.toRadians(0)), 1.0, 0.5);
//                .lineTo(new Pose(30, -30, Math.toRadians(180)), 0.4, 0.5)
//                .lineTo(new Pose(0, -30, Math.toRadians(90)), 0.4, 0.5)
//                .lineTo(new Pose(0, 0, Math.toRadians(0)), 0.4, 0.5);

        waitForStart();

        movement.run();
    }
}