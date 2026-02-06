package org.firstinspires.ftc.teamcode.utility.autonomous;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

public class Paths {
    public static class Blue {
        // Start
        public static Trajectory startLong = new Trajectory(FieldConstants.Start.blueLong);

        public static Trajectory startShort = new Trajectory(FieldConstants.Start.blueShort);
        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.blue3)
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true))
                .lineTo(FieldConstants.Spike.End.blue3)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.blue2)
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true))
                .lineTo(FieldConstants.Spike.End.blue2)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.blue1)
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true))
                .lineTo(FieldConstants.Spike.End.blue1)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .lineToPrecise(FieldConstants.Launch.blueLong);

        // Short Launch
        public static Trajectory launchShort = new Trajectory() //using static to indicate that an object is NOT required to use this method
                .lineToPrecise(FieldConstants.Launch.blueShort);

        // Gate
        public static Trajectory gate = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.blue)
                .lineToPrecise(FieldConstants.Gate.End.blue)
                .action(() -> {
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

        // End Long (Move out of launch zone)
        public static Trajectory endLong = new Trajectory()
                .lineTo(FieldConstants.End.blueLong);

        // End Short (Move away from goal)
        public static Trajectory endShort = new Trajectory()
                .lineTo(FieldConstants.End.blueShort);
    }

    public static class Red {
        // Start
        public static Trajectory startLong = new Trajectory(FieldConstants.Start.redLong);

        public static Trajectory startShort = new Trajectory(FieldConstants.Start.redShort);
        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.red3)
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true))
                .lineTo(FieldConstants.Spike.End.red3)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.red2)
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true))
                .lineTo(FieldConstants.Spike.End.red2)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.red1)
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.runIntake(true))
                .action(() -> Actuation.runTransfer(true))
                .lineTo(FieldConstants.Spike.End.red1)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .lineToPrecise(FieldConstants.Launch.redLong);

        // Short Launch
        public static Trajectory launchShort = new Trajectory()
                .lineToPrecise(FieldConstants.Launch.redShort);

        // Gate
        public static Trajectory gate = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.red)
                .lineToPrecise(FieldConstants.Gate.End.red)
                .action(() -> {
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

        // Park
        public static Trajectory park = new Trajectory()
                .lineToPrecise(FieldConstants.Park.blue);

        // End Long (Move out of launch zone)
        public static Trajectory endLong = new Trajectory()
                .lineTo(FieldConstants.End.redLong);

        // End Short (Move away from goal)
        public static Trajectory endShort = new Trajectory()
                .lineTo(FieldConstants.End.redShort);
    }
}
