package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="Autonomous")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        Trajectory movement = new Trajectory(new Pose(0, 0, Math.toRadians(0.0)))
                .lineTo(new Pose(10, 0, Math.toRadians(0)), 0.5, 0.5)
                .lineTo(new Pose(0, 0, Math.toRadians(0)), 0.5, 0.5);

        waitForStart();

        movement.run();
    }
}