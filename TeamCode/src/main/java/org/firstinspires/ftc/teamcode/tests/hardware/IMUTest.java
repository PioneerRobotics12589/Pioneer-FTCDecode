package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.localization.IMUControl;

@Config
@TeleOp(name="IMU Test", group="tests")
public class IMUTest extends OpMode {

    double imu_offset = 0.0;

    @Override
    public void init() {
//        Actuation.setup(hardwareMap, telemetry);
        IMUControl.setup(hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        IMUControl.setYaw(0);
    }

    @Override
    public void loop() {
//        OttoCore.updatePosition();
        telemetry.addData("IMU Heading", Math.toDegrees(IMUControl.getHeading()));
//        Actuation.packet.put("Odometry Heading", OttoCore.robotPose.heading);
    }
}
