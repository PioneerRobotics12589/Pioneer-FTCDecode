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



        waitForStart();
        Actuation.driveStarter(0,-0.5);
        sleep(190);
        Actuation.driveStarter(0,0);
        Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
        sleep(100);
        Actuation.setLoaders(true);
        sleep(5000);
        Actuation.setLoaders(false);
        Actuation.setFlywheel(0);
        Actuation.driveStarter(0,0.5);
        sleep(950);
        Actuation.driveStarter(-0.5, 0);
        sleep(2500);
        Actuation.driveStarter(0, 0);
        sleep(1500);
        Actuation.driveStarter(0.5,0);
        sleep(2100);
        Actuation.driveStarter(0,-0.5);
        sleep(660);
        //System.out.println("67 mustard gooner");
        //Pergie is very much gay
        //I am 67 ing it guys
        Actuation.driveStarter(0,0);
        Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
        sleep(500);
        Actuation.setLoaders(true);
        sleep(5000);
        Actuation.setLoaders(false);
        Actuation.setFlywheel(0);










    }
}