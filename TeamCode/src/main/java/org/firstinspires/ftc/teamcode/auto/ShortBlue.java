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
import org.firstinspires.ftc.teamcode.utility.imu.IMUControl;

@Autonomous(name="ShortBlue", group="Blue Auto")
public class ShortBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();

//        Paths.Blue.startShort.run();
        OttoCore.robotPose = FieldConstants.Start.blueShort;
        IMUControl.setYaw(OttoCore.robotPose.heading);
        Actuation.setFlywheel(1470);
        Actuation.turretMoveTowards(Math.toRadians(10));
        sleep(1000);

        Actuation.runIntake(true);
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Blue.launchShort.run();
        Actuation.packet.addLine("On the way to: Spike1");
        Actuation.updateTelemetry();

        Paths.Blue.spike1.run();
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Blue.launchShort.run();
        Actuation.packet.addLine("On the way to: Spike2");
        Actuation.updateTelemetry();

        Paths.Blue.spike2.run();
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Blue.launchShort.run();
        Actuation.packet.addLine("On the way to: Spike3");
        Actuation.updateTelemetry();

        Paths.Blue.spike3.run();
        Actuation.packet.addLine("On the way to: Launch Zone");
        Actuation.updateTelemetry();

        Paths.Blue.launchShort.run();
        Actuation.packet.addLine("On the way to: Gate");
        Actuation.updateTelemetry();

        Paths.Blue.gate.run();
        Actuation.packet.addLine("Auto Ended.");
        Actuation.updateTelemetry();
    }
}
