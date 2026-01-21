package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utility.autonomous.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDCoeffs;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.6;
        //        public static double moveSpeed = 0.2;
        public static double turnSpeed = 0.8;
//        public static double turnSpeed = 0.26667; // haha, 67
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 2000;
        public static double wheel_circ = 3.9566929; // inches circumference of dead wheels
        public static double track_width = 8.786584086675999; // inches distance between odo wheels
        public static double forward_offset = 9.5; // inches distance from center of robot to perp wheel
// 7, 9
        public static double launcherHeight = 0.25; // meters height from ground to launcher
        public static double flwheelRad = 0.1; // meters flywheel radius
    }

    @Config
    public static class Movement {
        public static PIDCoeffs lateralGains = new PIDCoeffs(0.2, 50, 0.08);
        public static PIDCoeffs verticalGains = new PIDCoeffs(0.25, 40, 0.09);
        public static PIDCoeffs rotationalGains = new PIDCoeffs(4.0, 30, 0.2);
    }

    @Config
    public static class Launcher {
        // Two Motors: P = 40, I = 3, D = 0
        public static PIDFCoefficients flywheelPID = new PIDFCoefficients(600, 60, 15, 0);

        public static double kp = 0, ki = 0, kd = 0;
        public static PIDController turretPIDAprilTag = new PIDController(kp, ki, kd); // Turret PID for rotating to towards an AprilTag
        // 1.5, 0, 0 for full speed
        public static PIDController turretPIDRot = new PIDController(0.25, 0, 0); // Turret PID for rotating to a specified position
        public static double turretTicks = 550; // 1467 / 24
        public static double turretRatio = 0.58965273271038066041666666666667; // 80:12 gear ratio 21 88
        public static double turretMaxAngle = Math.toRadians(180);
        public static double turretMaxAngleTicks = 1000; // 180 degrees for both rotations
        public static double turretOffset = -5; // Turret Offset from center of the robot

        public static int shortLaunch = 1467;
        public static int longLaunch = 1630;

        public static double targetHeight = 0.984;
        public static double artifactRadius = 0.12446;
    }

    @Config
    public static class Intake {
        public static double intakeSpeed = -0.85;
        public static double transferSpeed = -1.0;
    }

    @Config
    public static class LimelightConsts {
        // PID for pixel to heading
        private static final double kp_xh = 0.002, ki_xh = 0.0, kd_xh = 0.0;
        public static PIDController head_PID = new PIDController(new PIDCoeffs(kp_xh, ki_xh, kd_xh));
        public static int PIPELINE_APRILTAG = 0, PIPELINE_GREEN = 1, PIPELINE_PURPLE = 2;
        public static int RESOLUTION_X = 640, RESOLUTION_Y = 480;

    }
}
