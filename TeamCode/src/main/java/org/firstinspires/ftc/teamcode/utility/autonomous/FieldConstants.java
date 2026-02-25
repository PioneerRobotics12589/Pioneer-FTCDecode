package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class FieldConstants {

    // Start Positions
    @Config
    public static class Start {
        public static Pose redLong = new Pose(-65, -17.5, Math.toRadians(0.0));
        public static Pose redShort = new Pose(53.23, -47.2, Math.toRadians(-52.78));
        public static Pose blueLong = new Pose(-65, 17.5, Math.toRadians(0.0));
        public static Pose blueShort = new Pose(53.23, 47.2, Math.toRadians(52.78));
    }

    // Spike Positions
    @Config
    public static class Spike {
        // 1 - Closest to goal; 3 - Farthest from goal

        public static class Start {
            public static Pose red1 = new Pose(13.5, -23, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-15, -23, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-35.5, -23, Math.toRadians(-90.0));
            public static Pose red4 = new Pose(-52, -59.5, Math.toRadians(-130.0));
            public static Pose blue1 = new Pose(13.5, 23, Math.toRadians(90.0));
            public static Pose blue2 = new Pose(-13.5, 23, Math.toRadians(90.0));
            public static Pose blue3 = new Pose(-35.5, 23, Math.toRadians(90.0));
            public static Pose blue4 = new Pose(-52, 59.5, Math.toRadians(130.0));
        }

        public static class End {
            public static Pose red1 = new Pose(13.5, -49.5, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-15, -54, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-35.5, -54, Math.toRadians(-90.0));
            public static Pose red4 = new Pose(-60, -59.5, Math.toRadians(-163.0));
            public static Pose blue1 = new Pose(13.5, 49.5, Math.toRadians(90.0));
            public static Pose blue2 = new Pose(-13.5, 54, Math.toRadians(90.0)); //HelloWorld('print'):
            public static Pose blue3 = new Pose(-35.5, 54, Math.toRadians(90.0));
            public static Pose blue4 = new Pose(-60, 59.5, Math.toRadians(163.0));
        }
    }

    // Launch Positions
    @Config
    public static class Launch {
        public static Pose redLong = new Pose(-56.5, -11.5, Math.toRadians(-42.8));
        public static Pose redShort = new Pose(12, -9, Math.toRadians(-90.0));
        public static Pose blueLong = new Pose(-56.5, 11.5, Math.toRadians(42.8));
        public static Pose blueShort = new Pose(12, 9, Math.toRadians(90.0));
    }

    // Gate Positions
    @Config
    public static class Gate {
        // Gate Distance: 2.5
        public static class Start {
            public static Pose red = new Pose(0, -45, Math.toRadians(-90.0));
            public static Pose blue = new Pose(0.0, 45, Math.toRadians(90.0));
        }

        public static class End {
            public static Pose red = new Pose(0, -48, Math.toRadians(90.0));
            public static Pose blue = new Pose(0, 48, Math.toRadians(-90.0));
        }
    }

    // Park Positions
    public static class Park {
        public static Pose red = new Pose(-7.4, -49, Math.toRadians(-92));
        public static Pose blue = new Pose(-52.5, 8, Math.toRadians(-90));
    }

    // Goal Points
    @Config
    public static class Goal {
        // Aim for FTC logos
        public static Point red = new Point(72, -72);
        public static Point blue = new Point(72, 72);
    }

    // End Positions
    @Config
    public static class End {

        public static Pose redShort = new Pose(50, -45, Math.toRadians(-55.6));
        public static Pose blueShort = new Pose(50, 45, Math.toRadians(55.6));
        public static Pose redLong = new Pose(-60, -50, Math.toRadians(0));
        public static Pose blueLong = new Pose(-60, 50, Math.toRadians(0));
    }

    // Dump Scan Positions
    @Config
    public static class Dump {
        public static Pose red = new Pose(40, -60, Math.toRadians(90));
        public static Pose blue = new Pose(-40, 60, Math.toRadians(-90));
    }
}