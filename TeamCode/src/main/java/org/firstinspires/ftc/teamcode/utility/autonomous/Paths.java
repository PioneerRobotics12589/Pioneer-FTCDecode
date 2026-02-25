package org.firstinspires.ftc.teamcode.utility.autonomous;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

public class Paths {
    public static class Blue {

        // Spike 4
        public static Trajectory spike4 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.blue4)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.blue4)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true));

        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.blue3)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.blue3)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.blue2)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.blue2)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true));


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.blue1)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.blue1)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .addPeriodic(AutoLaunch::launchOperation)
                .lineTo(FieldConstants.Launch.blueLong)
                .action(() -> {
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

        public static Trajectory launchShort = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .addPeriodic(AutoLaunch::launchOperation)
                .lineTo(FieldConstants.Launch.blueShort)
                .action(() -> {
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

        // Gate
        public static Trajectory gate = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.blue);
//                .lineTo(FieldConstants.Gate.End.blue)
//                .action(() -> {
//                    try {
//                        sleep(2000);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                });

        // End Long (Move out of launch zone)
        public static Trajectory endLong = new Trajectory()
                .lineTo(FieldConstants.End.blueLong);

        // End Short (Move away from goal)
        public static Trajectory endShort = new Trajectory()
                .lineTo(FieldConstants.End.blueShort);
    }

    public static class Red {
        // Spike 4
        public static Trajectory spike4 = new Trajectory()
                .lineTo(FieldConstants.Spike.Start.red4)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.red4)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true));

        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .lineTo(FieldConstants.Spike.Start.red3)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.red3)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .lineTo(FieldConstants.Spike.Start.red2)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.red2)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true))
                .lineTo(FieldConstants.Spike.Start.red2);


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .lineTo(FieldConstants.Spike.Start.red1)
                .action(() -> Actuation.reverse(true))
                .lineTo(FieldConstants.Spike.End.red1)
                .action(() -> Actuation.reverse(false))
                .action(() -> Actuation.runIntake(true));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .lineTo(FieldConstants.Launch.redLong)
                .action(() -> Actuation.turretMoveTowards(Math.toRadians(-25.5)))
                .action(() -> {
                    try {
                        sleep(250);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .action(() -> Actuation.intake.setPower(-1.0))
                .action(() -> Actuation.transfer.setPower(-0.8))
                .action(() -> {
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));

        public static Trajectory launchShort = new Trajectory()
                .lineTo(FieldConstants.Launch.redShort)
                .action(() -> Actuation.turretMoveTowards(Math.toRadians(-45)))
                .action(() -> {
                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .action(() -> Actuation.intake.setPower(-1.0))
                .action(() -> Actuation.transfer.setPower(-0.8))
                .action(() -> {
                    try {
                        sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .action(() -> Actuation.runIntake(false))
                .action(() -> Actuation.runTransfer(false));

        // Gate
        public static Trajectory gate = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.red);
//                .lineTo(FieldConstants.Gate.End.blue)
//                .action(() -> {
//                    try {
//                        sleep(2000);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                });

        // End Long (Move out of launch zone)
        public static Trajectory endLong = new Trajectory()
                .lineTo(FieldConstants.End.redLong);

        // End Short (Move away from goal)
        public static Trajectory endShort = new Trajectory()
                .lineTo(FieldConstants.End.redShort);
    }
}
