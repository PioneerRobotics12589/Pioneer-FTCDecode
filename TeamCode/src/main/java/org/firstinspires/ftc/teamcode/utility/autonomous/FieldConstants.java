package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class FieldConstants {

    @Config
    public static class Start {
        public static Pose redLong = new Pose(-64.77, -10.62, Math.toRadians(-4.4));
        public static Pose redShort = new Pose(48.6, -52, Math.toRadians(-55.6));
        public static Pose blueLong = new Pose(-23.8, 63, Math.toRadians(0.0));
        public static Pose blueShort = new Pose(-44.0, -59.5, Math.toRadians(-54.0));
    }
    @Config
    public static class Spike {
        // 1 - Closest to goal; 3 - Farthest from goal

        public static class Start {
            public static Pose red1 = new Pose(9.3, -28, Math.toRadians(-92.0));
            public static Pose red2 = new Pose(-15.5, -28, Math.toRadians(-94.0));
            public static Pose red3 = new Pose(-35.75, -28, Math.toRadians(-88.7));
            public static Pose blue1 = new Pose(-30.5, 36, Math.toRadians(-88.7));
            public static Pose blue2 = new Pose(-30.5, 12.0, Math.toRadians(-90.0));
            public static Pose blue3 = new Pose(-30.5, -12.0, Math.toRadians(-90.0));
        }

        public static class End {
            public static Pose red1 = new Pose(9.3, -28, Math.toRadians(-92.0));
            public static Pose red2 = new Pose(-15.5, -28, Math.toRadians(-94.0));
            public static Pose red3 = new Pose(-35.75, -28, Math.toRadians(-88.7));
            public static Pose blue1 = new Pose(-30.5, 36, Math.toRadians(-88.7));
            public static Pose blue2 = new Pose(-30.5, 12.0, Math.toRadians(-90.0));
            public static Pose blue3 = new Pose(-30.5, -12.0, Math.toRadians(-90.0));
        }
    }

    @Config
    public static class Launch {
        public static Pose redLong = new Pose(-57, -9.5, Math.toRadians(-20));
        public static Pose redShort = new Pose(16.0, -17.6, Math.toRadians(-50.3));
        public static Pose blueLong = new Pose(-16.0, 56.0, Math.toRadians(-15.0));
        public static Pose blueShort = new Pose(-11.0, -14.0, Math.toRadians(-38));
    }

    @Config
    public static class Gate {
        // Gate Distance: 2.5
        public static Pose red = new Pose(-7.4, -49, Math.toRadians(-92));
        public static Pose blue = new Pose(-52.5, 8, Math.toRadians(-90));
    }

    @Config
    public static class Goal {
        // Aim for FTC logos
        public static Point red = new Point(72, -68);
        public static Point blue = new Point(72, 68);
    }

    @Config
    public static class Dump {
        public static Pose red = new Pose(40, 60, Math.toRadians(90));
        public static Pose blue = new Pose(-40, 60, Math.toRadians(-90));
    }
}
