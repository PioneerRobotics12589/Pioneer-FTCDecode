package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class FieldConstants {

    // Start Positions
    @Config
    public static class Start {
        public static Pose redLong = new Pose(-62.2, -18.8, Math.toRadians(0.0));
        public static Pose redShort = new Pose(54.12, -47.41, Math.toRadians(-48.0));
        public static Pose blueLong = new Pose(-62.2, 18.8, Math.toRadians(0.0));
        public static Pose blueShort = new Pose(53.4, 47.25, Math.toRadians(52.84));
        public static Pose blueShort1 = new Pose(41.89, 53.80, Math.toRadians(89.05));
    }

    // Spike Positions
    @Config
    public static class Spike {
        // 1 - Closest to goal; 3 - Farthest from goal

        public static class Start {
            public static Pose red1 = new Pose(13.63, -30.25, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-10.95, -30.25, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-35.3, -30.25, Math.toRadians(-90.0));
            public static Pose red4short = new Pose(-35.63, -59.8, Math.toRadians(-145.0));
            public static Pose red4long = new Pose(-52, -59.5, Math.toRadians(-130.0));
            public static Pose blue1 = new Pose(13.64, 30.25, Math.toRadians(90.0));
            public static Pose blue2 = new Pose(-10.95, 30.25, Math.toRadians(90.0));
            public static Pose blue3 = new Pose(-35.3, 30.25, Math.toRadians(90.0));
            public static Pose blue4short = new Pose(-35.63, 59.8, Math.toRadians(145.0));
            public static Pose blue4long = new Pose(-30.2, 59.5, Math.toRadians(130.0));
        }

        public static class End {
            public static Pose red1 = new Pose(13.64, -52.5, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-11.5, -54.5, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-35.3, -54.5, Math.toRadians(-90.0));
            public static Pose red4short = new Pose(-56.7, -62.8, Math.toRadians(-175.0));
            public static Pose red4long = new Pose(-30.2, -59.5, Math.toRadians(-130.0));
            public static Pose blue1 = new Pose(13.64, 52.5, Math.toRadians(90.0));
            public static Pose blue2 = new Pose(-11.5, 54.5, Math.toRadians(90.0)); //HelloWorld('print'):
            public static Pose blue3 = new Pose(-35.3, 54.5, Math.toRadians(90.0));
            public static Pose blue4short = new Pose(-56.7, 62.8, Math.toRadians(175.0));
            public static Pose blue4long = new Pose(-30.2, 59.5, Math.toRadians(130.0));
        }
    }

    // Launch Positions
    @Config
    public static class Launch {
        public static Pose redLong = new Pose(-61, -8.9, Math.toRadians(-20));
        public static Pose redShort = new Pose(11.1, -12.43, Math.toRadians(-111.29));
        public static Pose blueLong = new Pose(-61, 8.9, Math.toRadians(20));
        public static Pose blueShort = new Pose(11.1, 12.43, Math.toRadians(111.29));
    }

    // Gate Positions
    @Config
    public static class Gate {
        // Gate Distance: 2.5
        public static class Start {
            public static Pose red = new Pose(0, -40, Math.toRadians(-90.0));
            public static Pose blue = new Pose(0, 40, Math.toRadians(90.0));
        }

        public static class End {
            public static Pose red = new Pose(0, -51, Math.toRadians(-90.0));
            public static Pose blue = new Pose(0, 51, Math.toRadians(90.0));
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
        public static double redX = 62, redY = -58;
        public static double blueX = 62, blueY = 58;
        public static Point red = new Point(62, -58);
        public static Point blue = new Point(62, 58);
    }

    // End Positions
    @Config
    public static class End {

        public static Pose redShort = new Pose(46.85, -25.98, Math.toRadians(-50));
        public static Pose blueShort = new Pose(46.85, 25.98, Math.toRadians(50));
        public static Pose redLong = new Pose(-60, -36, Math.toRadians(-90));
        public static Pose blueLong = new Pose(-60, 36, Math.toRadians(90));
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