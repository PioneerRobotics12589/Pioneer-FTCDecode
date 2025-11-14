package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utility.autonomous.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDCoeffs;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.6;
        public static double turnSpeed = 0.8;
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 2000;

        public static double centerMultiplier = 0.3922; // responsible for move
        public static double lateralMultiplier = 2.499555; // responsible for turn
        public static double perpendicularMultiplier = -0.39446041777312446; // responsible for strafe

        public static double wheel_circ = 10.05; // cm
        public static double track_width = 11.25 * lateralMultiplier; // inches distance between drive wheels
        public static double forward_offset = 15.0; // inches distance from center of robot to perp wheel

        public static double scale = wheel_circ / ticksPerRev;
    }

    @Config
    public static class Movement {
        public static PIDCoeffs lateralGains = new PIDCoeffs(0.18, 45, 0.08); // Control for strafe
        public static PIDCoeffs verticalGains = new PIDCoeffs(0.25, 40, 0.09); // Control for move
        public static PIDCoeffs rotationalGains = new PIDCoeffs(3.0, 30, 0.2); // Control for turn
    }

    @Config
    public static class Launcher {
        public static PIDFCoefficients pidCoeffs = new PIDFCoefficients(280.0, 2.0, 0, 0);

        public static int shortLaunch = 1500;
        public static int longLaunch = 1800;
    }

    @Config
    public static class ModelPID {
        public static double kp_x = 0.0, ki_x = 0.0, kd_x = 0.0;
        public static double kp_y = 0.0, ki_y = 0.0, kd_y = 0.0;
        public static double kp_h = 0.0, ki_h = 0.0, kd_h = 0.0;

        public static PIDController vX_PID = new PIDController(new PIDCoeffs(kp_x, ki_x, kd_x));
        public static PIDController vY_PID = new PIDController(new PIDCoeffs(kp_y, ki_y, kd_y));
        public static PIDController head_PID = new PIDController(new PIDCoeffs(kp_h, ki_h, kd_h));
    }
}

