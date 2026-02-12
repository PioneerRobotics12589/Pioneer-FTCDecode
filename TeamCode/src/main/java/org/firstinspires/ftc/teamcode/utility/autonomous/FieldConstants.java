package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class FieldConstants {

    // Start Positions
    @Config
    public static class Start {
        public static Pose redLong = new Pose(-64.77, -10.62, Math.toRadians(-4.4));
        public static Pose redShort = new Pose(48.6, -52, Math.toRadians(-55.6));
        public static Pose blueLong = new Pose(-23.8, 63, Math.toRadians(0.0));
        public static Pose blueShort = new Pose(48.91, 47.42, Math.toRadians(52.24));
    }

    // Spike Positions
    @Config
    public static class Spike {
        // 1 - Closest to goal; 3 - Farthest from goal

        public static class Start {
            public static Pose red1 = new Pose(13.2, -30.5, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-14.1, -30.3, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-37.5, -30.3, Math.toRadians(-90));
            public static Pose blue1 = new Pose(11.1, 32.3, Math.toRadians(90));
            public static Pose blue2 = new Pose(-18.6, 32.3, Math.toRadians(90));
            public static Pose blue3 = new Pose(-38.9, 32.3, Math.toRadians(90.0));
        }

        public static class End {
            public static Pose red1 = new Pose(13.2, -49.3, Math.toRadians(-90.0));
            public static Pose red2 = new Pose(-14.1, -49.3, Math.toRadians(-90.0));
            public static Pose red3 = new Pose(-37.5, 49.3, Math.toRadians(-90));
            public static Pose blue1 = new Pose(11.1, 52, Math.toRadians(90));
            public static Pose blue2 = new Pose(-13.6, 52, Math.toRadians(90));
            public static Pose blue3 = new Pose(-37.4, 52, Math.toRadians(90));
        }
    }

    // Launch Positions
    @Config
    public static class Launch {
        public static Pose redLong = new Pose(-57, -9.5, Math.toRadians(-20));
        public static Pose redShort = new Pose(14.5, -15.9, Math.toRadians(-46.6));
        public static Pose blueLong = new Pose(-16.0, 56.0, Math.toRadians(-15.0));
        public static Pose blueShort = new Pose(13.64, 13.78, Math.toRadians(46.1));
    }

    // Gate Positions
    @Config
    public static class Gate {
        // Gate Distance: 2.5
        public static class Start {
            public static Pose red = new Pose(-12, -48, Math.toRadians(-90));
            public static Pose blue = new Pose(-52.5, 8, Math.toRadians(-90));
        }

        public static class End {
            public static Pose red = new Pose(-16.1, -55.5, Math.toRadians(-67));
            public static Pose blue = new Pose(-52.5, 8, Math.toRadians(-90));
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
        public static Point red = new Point(72, -68);
        public static Point blue = new Point(72, 68);
    }

    // End Positions
    @Config
    public static class End {

        public static Pose redShort = new Pose(30, -30, Math.toRadians(-55.6));
        public static Pose blueShort = new Pose(-30, -30, Math.toRadians(55.6));
        public static Pose redLong = new Pose(-60, -50, Math.toRadians(0));
        public static Pose blueLong = new Pose(-60, 50, Math.toRadians(0));
    }

    // Dump Scan Positions
    @Config
    public static class Dump {
        public static Pose red = new Pose(40, 60, Math.toRadians(90));
        public static Pose blue = new Pose(-40, 60, Math.toRadians(-90));
    }
}
