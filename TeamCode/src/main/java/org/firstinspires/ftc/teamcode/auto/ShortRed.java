package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;

@Autonomous(name="ShortRed", group="Red Auto")
public class ShortRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();

        Thread turretOp = AutoMovement.turretOperation("red");
        turretOp.start();
        AutoLaunch.launchThreadStart();

        Paths.Red.startShort.run();
        Paths.Red.launchLong.run();
        Paths.Red.spike1.run();
        Paths.Red.launchShort.run();
        Paths.Red.endShort.run();

        turretOp.interrupt();
        AutoLaunch.launchThreadStop();
    }
}
