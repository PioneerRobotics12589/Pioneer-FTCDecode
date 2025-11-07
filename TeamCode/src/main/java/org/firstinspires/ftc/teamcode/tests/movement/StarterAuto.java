package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="Starter Auto")
public class StarterAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        waitForStart();
        Actuation.driveStarter(0,20);
        sleep(150);
        Actuation.driveStarter(-1,0);
        sleep(200);
        Actuation.driveStarter(0,-20);
        sleep(450);
        Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
        sleep(500);
        Actuation.setLoaders(true);
        sleep(5000);
        Actuation.setLoaders(false);





    }
}