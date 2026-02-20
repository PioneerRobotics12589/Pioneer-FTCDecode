package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;
import org.firstinspires.ftc.teamcode.utility.imu.IMUControl;

@Autonomous(name="ShortRed", group="Red Auto")
public class ShortRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        OttoCore.robotPose = FieldConstants.Start.redShort;

        waitForStart();

//        Paths.Blue.startShort.run();
        IMUControl.setYaw(OttoCore.robotPose.heading);
        Actuation.setFlywheel(1470);
        Actuation.turretMoveTowards(Math.toRadians(-10));
        sleep(1000);

        Actuation.runIntake(true);
        Paths.Red.launchShort.run();
        Paths.Red.spike1.run();
        Paths.Red.launchShort.run();
        Paths.Red.spike2.run();
        Paths.Red.launchShort.run();
        Paths.Red.spike3.run();
        Paths.Red.launchShort.run();
        Paths.Red.gate.run();
    }
}
