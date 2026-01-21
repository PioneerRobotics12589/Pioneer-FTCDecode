package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.lang.reflect.Field;

@Autonomous(name="LongBlue", group = "Blue Auto")
@Config
public class LongBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();

        // FULL AUTO (18 artifacts? 6 teammate, 12 us)
//        preloads.run();
//        launch.run();
//        spike3.run();
//        launch2.run();
//        spike2.run();
//        launch3.run();
//        spike3end.run();
//        gate.run();
//        launch.run();
//        pickup_dump.run();
//        launch.run();
//        pickup_dump.run();
//        launch.run();
    }
}
