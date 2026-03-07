package org.firstinspires.ftc.teamcode.utility.autonomous;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Trajectory;

public class Paths {
    public static class Blue {

        // Spike 4
        public static Trajectory spike4long = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(-ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.25))
                .lineThrough(FieldConstants.Spike.Start.blue4long, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.End.blue4long, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.Start.blue4long, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.End.blue4long, 1.0, 1.0)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.transfer.setPower(0));

        public static Trajectory spike4short = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.25))
                .lineThrough(FieldConstants.Spike.Start.blue4short, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.End.blue4short, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.Start.blue4short, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.End.blue4short, 1.0, 1.0)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.transfer.setPower(0));

        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.setBlocker(true))
                .lineTo(FieldConstants.Spike.Start.blue3, 1.0, 1.0)
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.5))
                .lineTo(FieldConstants.Spike.End.blue3, 1.0, 1.0)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.transfer.setPower(0));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.setBlocker(true))
                .lineTo(FieldConstants.Spike.Start.blue2, 1.0, 1.0)
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.5))
                .lineTo(FieldConstants.Spike.End.blue2, 1.0, 1.0)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.transfer.setPower(0));


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.setBlocker(true))
                .lineTo(FieldConstants.Spike.Start.blue1, 1.0, 1.0)
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.5))
                .lineTo(FieldConstants.Spike.End.blue1, 1.0, 1.0)
                .action(() -> Actuation.setBlocker(false))
                .action(() -> Actuation.transfer.setPower(0));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.setBlocker(false))
                .lineTo(FieldConstants.Launch.blueLong)
                .launchOp()
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.75))
                .sleepWithPeriodics(1500)
                .action(() -> {
                    Actuation.runTransfer(false);
                });

        public static Trajectory launchShort = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.setBlocker(false))
                .lineTo(FieldConstants.Launch.blueShort)
                .launchOp()
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.75))
                .sleepWithPeriodics(1500)
                .action(() -> Actuation.runTransfer(false));

        // Gate
        public static Trajectory gateEnd = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.blue);
        public static Trajectory gateOpen = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.blue)
                .lineTo(FieldConstants.Gate.End.blue)
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
        // Spike 4
        public static Trajectory spike4long = new Trajectory()
                .action(() -> Actuation.intake.setPower(-ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed * 0.25))
                .lineThrough(FieldConstants.Spike.Start.red4long)
                .lineThrough(FieldConstants.Spike.End.red4long, 0.2, 1.0)
                .lineThrough(FieldConstants.Spike.Start.red4long, 0.2, 1.0)
                .lineThrough(FieldConstants.Spike.End.red4long, 0.2, 1.0)
                .action(() -> Actuation.transfer.setPower(0));

        public static Trajectory spike4short = new Trajectory()
                .action(() -> Actuation.intake.setPower(-ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed * 0.25))
                .lineTo(FieldConstants.Spike.Start.red4short)
                .lineThrough(FieldConstants.Spike.End.red4short, 0.2, 1.0)
                .lineThrough(FieldConstants.Spike.Start.red4short, 0.2, 1.0)
                .lineThrough(FieldConstants.Spike.End.red4short, 0.2, 1.0)
                .action(() -> Actuation.transfer.setPower(0));

        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.red3)
                .action(() -> Actuation.intake.setPower(-ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed * 0.25))
                .lineThrough(FieldConstants.Spike.End.red3, 0.2, 1.0)
                .action(() -> Actuation.transfer.setPower(0));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.red2)
                .action(() -> Actuation.intake.setPower(-ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed * 0.25))
                .lineThrough(FieldConstants.Spike.End.red2, 0.2, 1.0)
                .action(() -> Actuation.transfer.setPower(0));


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .lineThrough(FieldConstants.Spike.Start.red1)
                .action(() -> Actuation.intake.setPower(-ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed * 0.25))
                .lineThrough(FieldConstants.Spike.End.red1, 0.2, 1.0)
                .action(() -> Actuation.transfer.setPower(0));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .lineTo(FieldConstants.Launch.redLong)
                .launchOp()
                .action(() -> {
                    Actuation.runTransfer(true);
                    Actuation.runIntake(true);
                })
                .sleepWithPeriodics(1500)
                .action(() -> {
                    Actuation.runTransfer(false);
                    Actuation.runIntake(false);
                });

        public static Trajectory launchShort = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .lineTo(FieldConstants.Launch.redShort)
                .launchOp()
                .action(() -> {
                    Actuation.runTransfer(true);
                    Actuation.runIntake(true);
                })
                .sleepWithPeriodics(1500)
                .action(() -> {
                    Actuation.runTransfer(false);
                    Actuation.runIntake(false);
                });

        // Gate
        public static Trajectory gate = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.red);

        public static Trajectory gateOpen = new Trajectory()
                .lineTo(FieldConstants.Gate.Start.red)
                .lineTo(FieldConstants.Gate.End.red)
                .action(() -> {
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

        // End Long (Move out of launch zone)
        public static Trajectory endLong = new Trajectory()
                .lineTo(FieldConstants.End.redLong);

        // End Short (Move away from goal)
        public static Trajectory endShort = new Trajectory()
                .lineTo(FieldConstants.End.redShort);
    }
}
