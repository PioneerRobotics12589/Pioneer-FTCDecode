package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;

@Autonomous(name="ShortBlue", group="blue")
public class ShortBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();

        Thread turretOp = AutoMovement.turretOperation("blue");
        turretOp.start();
        AutoLaunch.launchThreadStart();

        Paths.Blue.startShort.run();
        Paths.Blue.launchLong.run();
        Paths.Blue.spike1.run();
        Paths.Blue.launchShort.run();
        Paths.Blue.endShort.run();

        turretOp.interrupt();
        AutoLaunch.launchThreadStop();
    }
}
