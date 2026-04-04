package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class FieldConstants {

    // Start Positions
    @Config
    public static class Start {
        public static Pose redLong = new Pose(-62.2, -18.8, Math.toRadians(0.0));
        public static Pose redShort = new Pose(51.5, -49.5, Math.toRadians(-56));
        public static Pose redShort1 = new Pose(40.88, -54.707, Math.toRadians(-90.89));
        public static Pose blueLong = new Pose(-62.2, 18.8, Math.toRadians(0.0));
        public static Pose blueShort = new Pose(53.4, 47.25, Math.toRadians(52.84));
        public static Pose blueShort1 = new Pose(41.4, 54.17, Math.toRadians(91.36));
    }

    // Spike Positions
    @Config
    public static class Spike {
        // 1 - Closest to goal; 3 - Farthest from goal

        public static class Start {
            public static Pose red1 = new Pose(15, -30.25, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-13.5, -30.25, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-35.3, -30.25, Math.toRadians(-90.0));
            public static Pose red4short = new Pose(-35.63, -64, Math.toRadians(-145.0));
            public static Pose red4long = new Pose(-35, -62, Math.toRadians(-143.3));
            public static Pose blue1 = new Pose(13.1, 30.25, Math.toRadians(90.0));
            public static Pose blue2 = new Pose(-11.95, 27.5, Math.toRadians(90.0));
            public static Pose blue3 = new Pose(-45.3, 25, Math.toRadians(90.0));
            public static Pose blue4short = new Pose(-35.63, 59.8, Math.toRadians(145.0));
            public static Pose blue4long = new Pose(-35, 62, Math.toRadians(143.3));
        }

        public static class End {
            public static Pose red1 = new Pose(15, -57, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-13.5, -62, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-35.3, -62, Math.toRadians(-90.0));
            public static Pose red4short = new Pose(-56.7, -62.8, Math.toRadians(-175.0));
            public static Pose red4long = new Pose(-62, -62, Math.toRadians(-158.0));
            public static Pose blue1 = new Pose(13.1, 45, Math.toRadians(90.0));
            public static Pose blue2 = new Pose(-12.5, 54, Math.toRadians(90.0)); //HelloWorld('print'):
            public static Pose blue3 = new Pose(-35.3, 54, Math.toRadians(90.0));
            public static Pose blue4short = new Pose(-56.7, 62.8, Math.toRadians(175.0));
            public static Pose blue4long = new Pose(-62, 62, Math.toRadians(158.0));
        }
    }

    // Launch Positions
    @Config
    public static class Launch {
        public static Pose redLong = new Pose(-58, -15, Math.toRadians(-20));
        public static Pose redShort = new Pose(11.1, -12.43, Math.toRadians(-111.29));
        public static Pose blueLong = new Pose(-58, 15, Math.toRadians(20));
        public static Pose blueShort = new Pose(11.1, 12.43, Math.toRadians(111.29));
    }

    // Gate Positions
    @Config
    public static class Gate {
        // Gate Distance: 2.5
        public static class Start {
            public static Pose red = new Pose(-1.5, -40, Math.toRadians(-90.0));
            public static Pose blue = new Pose(1.5, 40, Math.toRadians(90.0));
        }

        public static class End {
            public static Pose red = new Pose(-1.5, -55.5, Math.toRadians(-90.0));
            public static Pose blue = new Pose(1.5, 55.5, Math.toRadians(90.0));
        }
    }

    // Park Positions
    public static class Park {
        public static Pose red = new Pose(-39.85, -32.27, Math.toRadians(90));
        public static Pose blue = new Pose(-39.85, 32.27, Math.toRadians(-90));
    }

    // Goal Points
    @Config
    public static class Goal {
        // Aim for FTC logos
        public static double redX = 62, redY = -72;
        public static double blueX = 62, blueY = 62;
        public static Point red = new Point(62, -65);
        public static Point blue = new Point(62, 62);
    }

    // End Positions
    @Config
    public static class End {

        public static Pose redShort = new Pose(46.85, -25.98, Math.toRadians(-50));
        public static Pose blueShort = new Pose(46.85, 25.98, Math.toRadians(50));
        public static Pose redLong = new Pose(-60, -40, Math.toRadians(0));
        public static Pose blueLong = new Pose(-60, 40, Math.toRadians(0));
    }

    public static class Reset {
        public static Pose redCorner = new Pose(-62.13, -60.13, Math.toRadians(-90));
        public static Pose blueCorner = new Pose(-62.13, 60.3, Math.toRadians(90));
    }

    // Dump Positions
    @Config
    public static class Dump {
        public static Pose red = new Pose(40, -60, Math.toRadians(90));
        public static Pose blue = new Pose(-40, 60, Math.toRadians(-90));
    }
}