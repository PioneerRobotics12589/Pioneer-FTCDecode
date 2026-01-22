package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;

@Autonomous(name="LongBlue", group = "Blue Auto")
@Config
public class LongBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();

        AutoLaunch.launchThreadStart();

        // FULL AUTO (15 artifacts? 6 teammate, 9 us)
        Paths.Blue.startLong.run();
        Paths.Blue.launchLong.run();
        Paths.Blue.spike3.run();
        Paths.Blue.launchLong.run();
        Paths.Blue.spike2.run();
        Paths.Blue.launchLong.run();
        Paths.Blue.gate.run();

        AutoLaunch.launchThreadStop();
    }
}
