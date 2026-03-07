package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDCoeffs;
import org.firstinspires.ftc.teamcode.utility.dataTypes.SimpleMotorFeedforward;

public class ActuationConstants { // ryan pergola is a gay
    @Config
    public static class Autonomous {
        public static double moveSpeed = 1.0;
        //        public static double moveSpeed = 0.2;
        public static double turnSpeed = 1.0;
//        public static double turnSpeed = 0.26667; // haha, 67 i love ryan fagan
    }

    @Config
    public static class Drivetrain {
//        public static final double ticksPerRev = 2000;
//        public static double wheel_circ = 3.9566929; // inches circumference of dead wheels
        public static double track_width = 8.7637249881059645; // inches distance between odo wheels
        public static double forward_offset = 7.5; // inches distance from center of robot to perp wheel
        public static double launcherHeight = 0.4325; // meters height from ground to launcher
        public static double flwheelRad = 0.1; // meters flywheel radius
        public static double xOdoOffset = -4.38;
        public static double yOdoOffset = -7.5;
    }

    @Config
    public static class Movement {
        /*
        Base control systems for autonomous movement:
        PID: creates the essential movement speeds
        Feedforward: Adjusts the robot better due to the friction against the field (ks term)
         */
        public static PIDController lateralPID = new PIDController(0.075, 0, 0.012);
        public static SimpleMotorFeedforward lateralFF = new SimpleMotorFeedforward(0.13, 0.0, 0.0);
        public static PIDController verticalPID = new PIDController(0.075, 0.0, 0.012);
        public static SimpleMotorFeedforward verticalFF = new SimpleMotorFeedforward(0.13, 0.0, 0.0);
        public static PIDController rotationalPID = new PIDController(1.0, 0, 0.2);
        public static SimpleMotorFeedforward rotationalFF = new SimpleMotorFeedforward(0.13, 0.0, 0.0);
    }

    @Config
    public static class Launcher {
        // Two Motors: P = 40, I = 3, D = 0
        // Before using F: P = 600, I = 50, D = 50, F = 20
        public static PIDController flywheelPID = new PIDController(0.65, 0, 0.0002);
        public static SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(0.006, 0.00039, 0);
        // 1, 0.5, 0.003
        public static PIDController turretPID = new PIDController(0.9, 0.002, 0.0003);
        public static double turretTicks = 384.5; // Ticks per revolution on the turret input motor
        public static double turretRatio = 130.0 / 34.0 / (2*Math.PI); // 80:21 gear ratio
        public static double turretMaxAngle = AngleUnit.normalizeRadians(Math.toRadians(110));
        public static double turretFF = 0.01;
        public static double sotmAdjustMult = 1.0;
        public static double turretOffset = -2; // Turret offset from center of the robot (2 inches backwards from the center)
        public static int shortLaunch = 1340;
        public static int longLaunch = 1480;
        public static double targetHeight = 0.984;
        public static double artifactRadius = 0.12446;
    }

    @Config
    public static class Intake {
        public static double intakeSpeed = -1.0;
        public static double transferSpeed = 1.0;
        public static double blockerDown = 0.04;
        public static double blockerUp = 0.6;
    }

    @Config
    public static class LimelightConsts {
        // PID for pixel to heading
        public static double limelightHeight = 12;
        public static int PIPELINE_APRILTAG = 0, PIPELINE_GREEN = 1, PIPELINE_PURPLE = 2;
        public static int RESOLUTION_X = 640, RESOLUTION_Y = 480;

    }
}
