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
                //.action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                //.action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.blue4long, 1.0, 1.0)
                .lineToTime(FieldConstants.Spike.End.blue4long, 1.0, 1.0, 1000);
//                .lineToTime(FieldConstants.Spike.Start.blue4long, 1.0, 1.0, 500)
//                .lineToTime(FieldConstants.Spike.End.blue4long, 1.0, 1.0, 1000)
                //.action(() -> Actuation.transfer.setPower(0));

        public static Trajectory spike4short = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                //.action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                //.action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineThrough(FieldConstants.Spike.Start.blue4short, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.End.blue4short, 0.5, 1.0)
                .lineThrough(FieldConstants.Spike.Start.blue4short, 0.5, 1.0)
                .lineThrough(FieldConstants.Spike.End.blue4short, 0.5, 1.0);
                //.action(() -> Actuation.transfer.setPower(0));

        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                //.action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                //.action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.blue3, 1.0, 1.0)
                .lineToTime(FieldConstants.Spike.End.blue3, 1.0, 1.0, 2000);
                //.action(() -> Actuation.transfer.setPower(0));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                //.action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                //.action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.blue2, 1.0, 1.0)
                .lineTo(FieldConstants.Spike.End.blue2, 1.0, 1.0);
                //.action(() -> Actuation.transfer.setPower(0));


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                //.action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                //.action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.blue1, 1.0, 1.0)
                .lineTo(FieldConstants.Spike.End.blue1, 1.0, 1.0);
                //.action(() -> Actuation.transfer.setPower(0));


        public static Trajectory gateIntake = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
//                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
//                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineToTime(FieldConstants.Gate.Start.intakeBlue, 0.15, 0.15, 1000)
//                .lineToTime(FieldConstants.Gate.End.blue, 0.75, 0.6, 1000)
//                .action(() -> Actuation.drive(0.5, 0.0, 0.0))
                .sleepWithPeriodics(1000);
//                .action(() -> Actuation.transfer.setPower(0));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .addPeriodic(() -> Actuation.setFlywheel(1600))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .lineTo(FieldConstants.Launch.blueLong, 1.0, 1.0)
                //.action(() -> Actuation.setBlocker(false))
                .sleepWithPeriodics(500)
                .launchOp()
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.8))
                .sleepWithPeriodics(2000)
                .action(() -> Actuation.transfer.setPower(0.0));
                //.action(() -> Actuation.setBlocker(true))
                //.action(() -> Actuation.flywheel.setPower(0.8));

        public static Trajectory launchShort = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .addPeriodic(() -> Actuation.setFlywheel(1400))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .lineTo(FieldConstants.Launch.blueShort, 1.0, 1.0)
                //.action(() -> Actuation.setBlocker(false))
                .sleepWithPeriodics(500)
                .launchOp()
                .action(() -> Actuation.transfer.setPower(-1.0))
                .sleepWithPeriodics(800)
                //.action(() -> Actuation.runTransfer(false))
                //.action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.transfer.setPower(0.0));

        // Gate
        public static Trajectory gateEnd = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .lineTo(FieldConstants.Gate.Start.blue);
        public static Trajectory gateOpen = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .lineTo(FieldConstants.Gate.Start.blue)
                .lineTo(FieldConstants.Gate.End.blue, 1.0, 1.0)
                .sleepWithPeriodics(1000);

        // End Long (Move out of launch zone)
        public static Trajectory endLong = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .lineTo(FieldConstants.End.blueLong);

        // End Short (Move away from goal)
        public static Trajectory endShort = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .lineTo(FieldConstants.End.blueShort);
    }

    public static class Red {
        // Spike 4
        public static Trajectory spike4long = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
//                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.red4long, 1.0, 1.0)
                .lineToTime(FieldConstants.Spike.End.red4long, 1.0, 1.0, 1000)
//                .lineToTime(FieldConstants.Spike.Start.red4long, 1.0, 1.0, 750)
//                .lineToTime(FieldConstants.Spike.End.red4long, 1.0, 1.0, 1000)
                .action(() -> Actuation.transfer.setPower(0));

        public static Trajectory spike4short = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("blue"))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.5))
                .lineThrough(FieldConstants.Spike.Start.red4short, 1.0, 1.0)
                .lineThrough(FieldConstants.Spike.End.red4short, 0.5, 1.0)
                .lineThrough(FieldConstants.Spike.Start.red4short, 0.5, 1.0)
                .lineThrough(FieldConstants.Spike.End.red4short, 0.5, 1.0)
                .action(() -> Actuation.transfer.setPower(0));

        // Spike 3
        public static Trajectory spike3 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.red3, 1.0, 1.0)
                .lineToTime(FieldConstants.Spike.End.red3, 1.0, 1.0, 2000)
                .action(() -> Actuation.transfer.setPower(0));

        // Spike 2
        public static Trajectory spike2 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.red2, 1.0, 1.0)
                .lineTo(FieldConstants.Spike.End.red2, 1.0, 1.0)
                .action(() -> Actuation.transfer.setPower(0));


        // Spike 1
        public static Trajectory spike1 = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                .lineTo(FieldConstants.Spike.Start.red1, 1.0, 1.0)
                .lineTo(FieldConstants.Spike.End.red1, 1.0, 1.0)
                .action(() -> Actuation.transfer.setPower(0));

        public static Trajectory gateIntake = new Trajectory()
                    .addPeriodic(() -> AutoMovement.turretOperation("red"))
                    .action(() -> Actuation.setBlocker(true))
                    .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                    .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.6))
                    .lineTo(FieldConstants.Gate.Start.intakeRed, 1.0, 1.0)
                    .sleepWithPeriodics(1000)
                    .action(() -> Actuation.transfer.setPower(0));

        // Long Launch
        public static Trajectory launchLong = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .addPeriodic(() -> Actuation.setFlywheel(1470))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .lineTo(FieldConstants.Launch.redLong, 0.8, 1.0)
                .action(() -> Actuation.setBlocker(false))
                .sleepWithPeriodics(500)
                .launchOp()
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed*0.8))
                .sleepWithPeriodics(2000)
                .action(() -> Actuation.runTransfer(false))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.flywheel.setPower(0.8));

        public static Trajectory launchShort = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .addPeriodic(() -> Actuation.setFlywheel(1240))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .lineTo(FieldConstants.Launch.redShort, 1.0, 1.0)
                .action(() -> Actuation.setBlocker(false))
                .sleepWithPeriodics(500)
                .launchOp()
                .action(() -> Actuation.transfer.setPower(-ActuationConstants.Intake.transferSpeed))
                .sleepWithPeriodics(1750)
                .action(() -> Actuation.runTransfer(false))
                .action(() -> Actuation.setBlocker(true))
                .action(() -> Actuation.flywheel.setPower(0.6));

        // Gate
        public static Trajectory gateEnd = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .lineTo(FieldConstants.Gate.Start.red);
        public static Trajectory gateOpen = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .lineTo(FieldConstants.Gate.Start.red)
                .lineTo(FieldConstants.Gate.End.red)
                .sleepWithPeriodics(1000);

        // End Long (Move out of launch zone)
        public static Trajectory endLong = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .lineTo(FieldConstants.End.redLong);

        // End Short (Move away from goal)
        public static Trajectory endShort = new Trajectory()
                .addPeriodic(() -> AutoMovement.turretOperation("red"))
                .lineTo(FieldConstants.End.redShort);
    }
}
