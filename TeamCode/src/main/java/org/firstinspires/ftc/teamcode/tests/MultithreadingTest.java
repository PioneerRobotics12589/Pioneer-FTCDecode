package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

@TeleOp(name="Auto Multithreading Test")
public class MultithreadingTest extends OpMode {

    Thread transferOp = new Thread(() -> {
        while (true) {
            Actuation.runTransfer(gamepad1.right_trigger > 0.5, gamepad1.right_bumper || gamepad1.left_bumper);
            Actuation.reverse(gamepad1.left_trigger > 0.5);
        }
    });

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        transferOp.start();
    }

    @Override
    public void loop() {
        telemetry.addLine("X=" + OttoCore.robotPose.x + " Y=" + OttoCore.robotPose.y + "θ=" + Math.toDegrees(OttoCore.robotPose.heading));
        telemetry.addData("Is in launch zone", AutoLaunch.inLaunchZone());

        Actuation.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        Actuation.runIntake(gamepad1.right_trigger > 0.5);

        OttoCore.updatePosition();

        telemetry.update();
    }

    @Override
    public void stop() {
        transferOp.interrupt();
    }
}
