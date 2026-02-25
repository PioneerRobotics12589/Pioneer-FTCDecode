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

@Autonomous(name="LongBlue", group = "Blue Auto")
@Config
public class LongBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);
        OttoCore.robotPose = FieldConstants.Start.blueLong;
        IMUControl.setYaw(OttoCore.robotPose.heading);

        waitForStart();

        // FULL AUTO (15 artifacts? 6 teammate, 9 us)

        Actuation.setFlywheel(1550);
        Actuation.turretMoveTowards(Math.toRadians(-17.5));
        sleep(1750);

        Paths.Blue.launchLong.run();
        Paths.Blue.spike4.run();
        Paths.Blue.launchLong.run();
        Paths.Blue.spike3.run();
//        Paths.Blue.launchLong.run();
//        Paths.Blue.spike2.run();
        Paths.Blue.launchLong.run();
        Paths.Blue.gate.run();
//        Paths.Blue.endLong.run();
    }
}
