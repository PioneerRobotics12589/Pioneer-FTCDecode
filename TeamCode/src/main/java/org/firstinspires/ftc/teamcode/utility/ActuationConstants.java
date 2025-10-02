package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

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
        public static double lateralMultiplier = 2.5171; // responsible for turn
        public static double perpendicularMultiplier = -0.3923; // responsible for strafe

        public static double wheel_circ = 10.05; // cm
        public static double track_width = 11.25 * lateralMultiplier; // inches distance between drive wheels
        public static double forward_offset = 15.0; // inches distance from center of robot to perp wheel

        public static double scale = wheel_circ / ticksPerRev;
    }

    @Config
    public static class Movement {
        public static PIDCoeffs verticalGains = new PIDCoeffs(0.25, 0.0, 0.03); // Control for move
        public static PIDCoeffs lateralGains = new PIDCoeffs(0.0, 0.0, 0.0); // Control for strafe
        public static PIDCoeffs rotationalGains = new PIDCoeffs(4.0, 0.0, 0.1); // Control for turn
    }
}