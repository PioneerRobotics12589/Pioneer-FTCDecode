package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class FieldConstants {
    @Config
    public static class Start {
        public static Pose redLong = new Pose(23.8, 63, 0.0);
        public static Pose redShort = new Pose(44.0, -59.5, 54.0);
        public static Pose blueLong = new Pose(-23.8, 63, 0.0);
        public static Pose blueShort = new Pose(-44.0, -59.5, -54.0);
    }
    @Config
    public static class Spike {
        public static Pose red1 = new Pose(30.5, 36, 90.0);
        public static Pose red2 = new Pose(30.5, 12.0, 90.0);
        public static Pose red3 = new Pose(30.5, -12.0, 90.0);
        public static Pose blue1 = new Pose(-30.5, 36, -90.0);
        public static Pose blue2 = new Pose(-30.5, 12.0, -90.0);
        public static Pose blue3 = new Pose(-30.5, -12.0, -90.0);


    }

    @Config
    public static class Launch {
        public static Pose redLong = new Pose(16.0, 56.0, 15.0);
        public static Pose redShort = new Pose(12.0, -23.0, 46.0);
        public static Pose blueLong = new Pose(-16.0, 56.0, -15.0);
        public static Pose blueShort = new Pose(-12.0, -23.0, -46);
    }

    @Config
    public static class Goal {
        public static Point red = new Point(72, 72);
        public static Point blue = new Point(-72, 72);
    }
}
