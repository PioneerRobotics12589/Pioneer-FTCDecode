package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.Astar.Astar;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Disabled
@Autonomous(name="Pathfinding Test", group="tests")
public class PathFindingTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        Trajectory movement = new Trajectory(new Pose(0, 0, Math.toRadians(0)));
        for(Pose p : Astar.pathFind(0,0,35,0)){
            movement.lineTo(p);
        }

        waitForStart();
        movement.run();
    }
}