package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;
import org.firstinspires.ftc.teamcode.utility.localization.IMUControl;

@Autonomous(name="LongRed", group = "Red Auto")
@Config
public class LongRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);
        OttoCore.setup(hardwareMap);
        AutoLaunch.setTeam("red");

        waitForStart();

        OttoCore.setPose(FieldConstants.Start.redLong);


        Paths.Red.launchLong.run();

        Paths.Red.spike3.run();

        Paths.Red.launchLong.run();

        Paths.Red.spike2.run();

        Paths.Red.launchLong.run();

        Paths.Red.gateEnd.run();

//        Paths.Red.endLong.run();
    }
}
