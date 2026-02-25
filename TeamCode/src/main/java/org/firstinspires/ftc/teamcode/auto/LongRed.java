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
        OttoCore.robotPose = FieldConstants.Start.redLong;
        IMUControl.setYaw(OttoCore.robotPose.heading);

        waitForStart();

        // FULL AUTO (15 artifacts? 6 teammate, 9 us)

        Actuation.setFlywheel(1550);
        Actuation.turretMoveTowards(Math.toRadians(17.5));
        sleep(2000);

        Paths.Red.launchLong.run();
        Paths.Red.spike4.run();
        Paths.Red.launchLong.run();
        Paths.Red.spike3.run();
//        Paths.Red.launchLong.run();
//        Paths.Red.spike2.run();
        Paths.Red.launchLong.run();
        Paths.Red.gate.run();
//        Paths.Red.endLong.run();
    }
}
