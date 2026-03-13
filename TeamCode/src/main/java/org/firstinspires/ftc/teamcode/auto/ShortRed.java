package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Paths;
import org.firstinspires.ftc.teamcode.utility.localization.IMUControl;

@Autonomous(name="ShortRed", group="Red Auto")
public class ShortRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        OttoCore.setup(hardwareMap);
        AutoLaunch.setTeam("red");

        waitForStart();

        Actuation.setFlywheel(1250);
        sleep(400);
        Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed);

        OttoCore.setPose(FieldConstants.Start.redShort);


        Paths.Red.launchShort.run();

        Paths.Red.spike1.run();

        Paths.Red.gateOpen.run();

        Paths.Red.launchShort.run();

        Paths.Red.spike2.run();

        Paths.Red.launchShort.run();

        Paths.Red.spike3.run();

        Paths.Red.launchShort.run();

        Paths.Red.gateEnd.run();
    }
}
