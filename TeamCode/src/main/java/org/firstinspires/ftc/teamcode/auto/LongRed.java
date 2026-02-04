package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;

@Autonomous(name="LongRed", group = "Red Auto")
@Config
public class LongRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();

        AutoLaunch.launchThreadStart();

        // FULL AUTO (15 artifacts? 6 teammate, 9 us)
        Paths.Red.startLong.run();
        Paths.Red.launchLong.run();
        Paths.Red.spike3.run();
        Paths.Red.launchLong.run();
        Paths.Red.spike2.run();
        Paths.Red.launchLong.run();
        Paths.Red.gate.run();

        AutoLaunch.launchThreadStop();
    }
}
