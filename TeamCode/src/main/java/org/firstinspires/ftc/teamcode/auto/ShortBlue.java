package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

@Autonomous(name="ShortBlue", group="Blue Auto")
public class ShortBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        OttoCore.setPose(FieldConstants.Start.blueShort);

        waitForStart();

        Actuation.setFlywheel(1470);
        Actuation.turretMoveTowards(Math.toRadians(10));
        sleep(1000);

        Paths.Blue.launchShort.run();
        Paths.Blue.spike1.run();
        Paths.Blue.launchShort.run();
        Paths.Blue.spike2.run();
        Paths.Blue.launchShort.run();
        Paths.Blue.spike3.run();
//        Paths.Blue.launchShort.run();
//        Paths.Blue.gate.run();
    }
}
