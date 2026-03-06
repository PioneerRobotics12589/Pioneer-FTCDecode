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

        waitForStart();

        AutoLaunch.setTeam("blue");
        OttoCore.setPose(FieldConstants.Start.blueShort);

        Paths.Blue.launchShort.run();
        telemetry.addLine("Launch Complete.");
        telemetry.update();

//        Paths.Blue.spike1.run();
//        telemetry.addLine("Spike 1 Complete.");
//        telemetry.update();

//        Paths.Blue.launchShort.run();
//        telemetry.addLine("Launch Complete.");
//        telemetry.update();
//
//        Paths.Blue.spike2.run();
//        telemetry.addLine("Spike 2 Complete.");
//        telemetry.update();
//
//        Paths.Blue.launchShort.run();
//        telemetry.addLine("Launch Complete.");
//        telemetry.update();
//
//        Paths.Blue.spike3.run();
//        telemetry.addLine("Spike 3 Complete.");
//        telemetry.update();
//
//        Paths.Blue.launchShort.run();
//        telemetry.addLine("Launch Complete.");
//        telemetry.update();
//
////        Paths.Blue.launchShort.run();
//        Paths.Blue.gateEnd.run();
//        telemetry.addLine("Gate Complete.");
//        telemetry.update();
    }
}
