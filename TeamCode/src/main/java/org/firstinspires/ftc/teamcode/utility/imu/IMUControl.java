package org.firstinspires.ftc.teamcode.utility.imu;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUControl {
    private static IMU imu;
    private static double offset;

    private static boolean initialized = false;

    public static void setup(HardwareMap map, RevHubOrientationOnRobot.LogoFacingDirection logo_dir, RevHubOrientationOnRobot.UsbFacingDirection usb_dir) {
        imu = map.get(IMU.class, "imu"); // NEVER NEEDS TO BE CHANGED

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logo_dir, usb_dir);

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        initialized = true;
    }

    public static boolean isInitialized() {
        return initialized;
    }

    public static void setYaw(double newYaw) {
        offset = newYaw - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public static double getHeading() {
        return AngleUnit.normalizeRadians(offset + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}
