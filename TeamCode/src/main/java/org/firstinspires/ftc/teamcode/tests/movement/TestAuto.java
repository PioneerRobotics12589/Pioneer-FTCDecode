package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="Test Auto", group="tests")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);


        Trajectory movement = new Trajectory(new Pose(0, 0, Math.toRadians(0)))
                .lineTo(new Pose(0, 30, Math.toRadians(90)))
                .lineTo(new Pose(30, 30, Math.toRadians(180)))
                .lineTo(new Pose(30, 0, Math.toRadians(270)))
                .lineTo(new Pose(0, 0, Math.toRadians(0)));

        waitForStart();

        for (int i = 0; i < 5; i++)
            movement.run();
    }
}
