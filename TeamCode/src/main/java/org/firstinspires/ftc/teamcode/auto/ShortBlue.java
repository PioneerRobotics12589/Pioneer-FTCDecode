package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

@Autonomous(name="ShortBlue", group="blue")
public class ShortBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        Trajectory start = new Trajectory(FieldConstants.Start.blueShort)
                .action(() -> sleep(500));
        Trajectory launchShort = new Trajectory()
                .lineTo(FieldConstants.Launch.blueShort)
                .action(() -> Actuation.reverse(true))
                .action(() -> sleep(500))
                .action(() -> Actuation.reverse(false));
        Trajectory spike1Start = new Trajectory(FieldConstants.Spike.Start.blue1);
        Trajectory spike1End = new Trajectory(FieldConstants.Spike.End.blue1);

        waitForStart();

//        Thread turretOp = AutoMovement.turretOperation("blue");
//        turretOp.start();

        start.run();
        Actuation.setFlywheel(1500);
        launchShort.run();
        spike1Start.run();
        spike1End.run();
        launchShort.run();

//        Paths.Blue.startShort.run();
//        Actuation.setFlywheel(1500);
//        Paths.Blue.launchShort.run();
//        Paths.Blue.launch.run();
//        AutoLaunch.launchThreadStart();
//        Paths.Blue.spike1.run();
//        Paths.Blue.launchShort.run();
//        Paths.Blue.launch.run();
//        Paths.Blue.spike2.run();
//        Paths.Blue.launchShort.run();
//        Paths.Blue.launch.run();
//        Paths.Blue.spike3.run();
//        Paths.Blue.launchShort.run();
//        Paths.Blue.launch.run();
        //Paths.Blue.gate.run();

//        turretOp.interrupt();
//        AutoLaunch.launchThreadStop();
    }
}
