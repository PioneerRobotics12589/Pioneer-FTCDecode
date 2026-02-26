package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

@Disabled
@TeleOp(name="Auto Launch Test")
public class AutoLaunchTest extends OpMode {

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        OttoCore.updatePosition();
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
        AutoLaunch.updateAutoLaunchMobile(OttoCore.robotPose);
        AutoLaunch.setFlywheel();
        AutoLaunch.rotate();
    }
}
