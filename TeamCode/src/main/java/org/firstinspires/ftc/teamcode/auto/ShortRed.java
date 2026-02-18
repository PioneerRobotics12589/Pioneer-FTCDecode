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

        Paths.Red.startShort.run();
        Actuation.setFlywheel(1460);
        sleep(500);

        Actuation.runIntake(true);
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Red.launchShort.run();
        Actuation.packet.addLine("On the way to: Spike1");
        Actuation.updateTelemetry();

        Paths.Red.spike1.run();
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Red.launchShort.run();
        Actuation.packet.addLine("On the way to: Spike2");
        Actuation.updateTelemetry();

        Paths.Red.spike2.run();
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Red.launchShort.run();
        Actuation.packet.addLine("On the way to: Spike3");
        Actuation.updateTelemetry();

        Paths.Red.spike3.run();
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Red.launchShort.run();
        Actuation.packet.addLine("On the way to: Gate");
        Actuation.updateTelemetry();

        Paths.Red.gate.run();
        Actuation.packet.addLine("Auto Ended.");
        Actuation.updateTelemetry();
    }
}
