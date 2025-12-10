package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name = "Awe(sigma) Sauce")
@Config
public class RobotTeleOp extends OpMode {
    FtcDashboard dashboard;
    //public static int flywheelVel = 0;
    //public static boolean isTesting = true, servos = false;

    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();
    }

    public void loop() {
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

        if (gamepad1.right_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);
        }
        else if (gamepad1.left_bumper) {
            Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
            Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);
        }
        else {
            Actuation.setFlywheel(0);
            Actuation.checkFlywheelSpeed(gamepad1, 0);
        }
        Actuation.setIntake(gamepad1.right_trigger > 0.5);
        Actuation.reverseIntake(gamepad1.dpad_down);
        Actuation.setTransfer(gamepad1.left_trigger > 0.5);



        Actuation.updateTelemetry();

            //        if (!isTesting) {
//            if (gamepad1.right_bumper) {
//                Actuation.setFlywheel(ActuationConstants.Launcher.longLaunch);
//                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.longLaunch);
//            } else if (gamepad1.left_bumper) {
//                Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch);
//                Actuation.checkFlywheelSpeed(gamepad1, ActuationConstants.Launcher.shortLaunch);
//            }
//            Actuation.setFlywheel(0);
//            Actuation.checkFlywheelSpeed(gamepad1, 0);
//            Actuation.setLoaders(gamepad1.right_trigger > 0.5);
//            Actuation.packet.addLine("Calculated AngularVel: " + Actuation.launchVals("Blue")[1]);
//        } else {
//            Actuation.setFlywheel(flywheelVel);
//            if (Math.abs(Actuation.flywheel.getVelocity() - flywheelVel) <= 20) {
//                Actuation.packet.addLine("READY TO SHOOT");
//            } else {
//                Actuation.packet.addLine("FLYWHEEL NOT READY");
//            }
//            Actuation.updateTelemetry();
//            Actuation.setLoaders(servos);
//        }
//
//        if (gamepad1.a) {
//            Pose robotPose = new Pose(OttoCore.robotPose);
//            double[] flywheelVals = Actuation.launchVals("blue");
//            Pose targetPose = new Pose(robotPose.x, robotPose.y, flywheelVals[0]);
//
//            OttoCore.moveTowards(targetPose, 1.0, 1.0);
//
//            Actuation.setFlywheel(flywheelVals[1]);
//        }
    }
}