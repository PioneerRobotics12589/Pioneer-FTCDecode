package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.utility.Actuation.runIntake;
import static org.firstinspires.ftc.teamcode.utility.Actuation.runTransfer;
import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;
import static org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore.getMove;
import static org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore.getStrafe;
import static org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore.rotational;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.cameraVision.AprilTagDetection;
import org.firstinspires.ftc.teamcode.utility.cameraVision.ArtifactDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.lang.reflect.Field;
import java.util.List;

@Config
public class AutoMovement {

    //    public static void main(String[] args) {
//        System.out.println("Ryan Pergola mad cute :)");
//    }

    public static final double closestLaunchDist = 30; // way to many kewords man, consider changing to: double closestLaunchDIst
    /**
     * Aligns robot with colored artifact
     * @param color desired artifact color
     */
    public static void alignToArtifact(String color, double move, double strafe) {
        double turnRate = ArtifactDetection.trackArtifact(color);
        Actuation.drive(move, turnRate, -strafe);
        telemetry.addData("turnRate", turnRate);
    }

    /**
     * Automatically aligns robot with colored artifact
     * @param color desired artifact color
     */
    public static void alignToArtifactAuto(String color) {
        double forwardDist = 10.0; // Move 10" forward when locked on an artifact
        double turnRate;
        do {
            OttoCore.updatePosition();
            turnRate = ArtifactDetection.trackArtifact(color);
            Pose targetPose = OttoCore.relativeTransform(OttoCore.robotPose, forwardDist, 0.0, 0.0);
            Actuation.drive(getMove(targetPose, 0.8), turnRate, getStrafe(targetPose, 0.8));
        } while (Math.abs(turnRate) > 0.1);
    }

    /**
     * Autonomously takes the shortest path to the long-launch zone
     * @param team team color
     * Desmos Graph: <a href="https://www.desmos.com/calculator/rzvjeeb3be">...</a>
     */
    public static Pose closestLongLaunch(String team) {
        Point goal = team.equalsIgnoreCase("red") ? FieldConstants.Goal.red : FieldConstants.Goal.blue;
        Pose curr = new Pose(OttoCore.robotPose);
        Pose closest = new Pose(0, 0, 0);

        if (AutoLaunch.inLaunchZone()) {
            return curr;
        }

        double minX = -16, maxX = 16; // Constants so that we don't go to a position that will make us hit a wall or goal

        double x1 = (curr.x - curr.y - 48.0) / 2.0;
        double x2 = (curr.x + curr.y + 48.0) / 2.0;

        if (x1 >= 0) {
            closest.x = x1;
        } else if (x2 <= 0) {
            closest.x = x2;
        }

        closest.x = Math.max(minX, Math.min(maxX, closest.x));

        closest.y = -Math.abs(closest.x) - 48;

        closest.heading = Math.atan2(goal.y - closest.y, goal.x - closest.x);

        return closest;
    }

    /**
     * Autonomously takes the shortest path to the short-launch zone
     */
    public static Pose closestShortLaunch(String team) {
        Point goal = team.equalsIgnoreCase("red") ? FieldConstants.Goal.red : FieldConstants.Goal.blue;
        Pose curr = new Pose(OttoCore.robotPose);
        Pose closest = new Pose(0, 0, 0);

        if (AutoLaunch.inLaunchZone()) {
            return curr;
        }

        double minX = -45, maxX = 45; // Constants so that we don't go to a position that will make us hit a wall or goal

        double x1 = (curr.x - curr.y - 1.25) / 2.0;
        double x2 = (curr.x + curr.y + 1.25) / 2.0;

        if (x1 <= 0) {
            closest.x = x1;
        } else if (x2 >= 0) {
            closest.x = x2;
        }

        closest.x = Math.min(closestLaunchDist, Math.max(closest.x, -closestLaunchDist));

        closest.x = Math.max(minX, Math.min(maxX, closest.x));

        closest.y = Math.abs(closest.x) - 1.25;

        closest.heading = Math.atan2(goal.y - closest.y, goal.x - closest.x);

        return closest;
    }

    /**
     * Launches artifacts
     */
    public static void launch() throws InterruptedException {
        sleep(1800);

        // Push artifacts into flywheel
        Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed*0.8);
        Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed*0.8);

        // Launch for 1.5s
        sleep(2200);

        Actuation.setFlywheel(0);
        Actuation.runIntake(false);
        Actuation.runTransfer(false, true);
    }

    public static void launch(int speedup_millis) throws InterruptedException {
        sleep(speedup_millis);

        // Push artifacts into flywheel
        Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed*0.8);
        Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed*0.8);

        // Launch for 1.5s
        sleep(2200);

        Actuation.setFlywheel(0);
        Actuation.runIntake(false);
        Actuation.runTransfer(false, true);
    }

    /**
     * Creates a thread for operating the turret (auto-adjusting towards goal & maintaining flywheel velocity)
     * @param team team color
     * @return thread for turret operation
     */
    public static Thread turretOperation(String team) {
        return new Thread(() -> {
            while (true) {
                Pose robotPos = new Pose(OttoCore.robotPose);
                Pose reference = new Pose(robotPos);

                List<LLResultTypes.FiducialResult> fids = AprilTagDetection.getFiducials();
                int goalID = team.equals("red") ? 20 : 24;
                Pose fiducialGlobalPos = new Pose(0, 0, 0);
                int fidNum = 0; // Number of fiducials
                boolean trackingAprilTag = false;
                for (LLResultTypes.FiducialResult fid : fids) {
                    // Track AprilTag using center
//                    if (fid.getFiducialId() == goalID) {
//                        trackingAprilTag = true;
//
//                        fiducialGlobalPos.x = fid.getRobotPoseFieldSpace().getPosition().x;
//                        fiducialGlobalPos.y = fid.getRobotPoseFieldSpace().getPosition().y;
//                        fiducialGlobalPos.heading = (fid.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.RADIANS) + 2 * Math.PI) % (2 * Math.PI);
//
//                        // Track using AprilTag data if the goal was detected
//                        double yawDist = fid.getTargetXDegrees();
//                        double turretSignal = ActuationConstants.Launcher.turretPIDAprilTag.calculateSignal(0, yawDist);
//                        if (turretSignal > 0.05) { // Clip to stop vibrations
//                            Actuation.turret.setPower(turretSignal);
//                        }
//                        reference = OttoCore.relativeTransform(fiducialGlobalPos, ActuationConstants.Launcher.turretOffset, 0, 0);
//                        AutoLaunch.updateAutoLaunchS(team, reference); // Assuming static launching
//                        Actuation.setFlywheel(AutoLaunch.getTargetVel());
//                    }

                    // Track AprilTag using global positioning
                    if (fid.getFiducialId() == 20 || fid.getFiducialId() == 24) {
                        trackingAprilTag = true;
                        fidNum++;
                        fiducialGlobalPos.x += fid.getRobotPoseFieldSpace().getPosition().x;
                        fiducialGlobalPos.y += fid.getRobotPoseFieldSpace().getPosition().y;
                        fiducialGlobalPos.heading += (fid.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.RADIANS) + 2 * Math.PI) % (2 * Math.PI);

                        reference = new Pose(fiducialGlobalPos.x / fidNum, fiducialGlobalPos.y / fidNum, fiducialGlobalPos.heading / fidNum);
                    }
                }

                if (!trackingAprilTag) {
                    reference = new Pose(OttoCore.robotPose);
                }

                reference = OttoCore.relativeTransform(reference, ActuationConstants.Launcher.turretOffset, 0, 0);

//                AutoLaunch.updateAutoLaunchM(team, reference); // Assuming mobile or static launching
                AutoLaunch.updateAutoLaunchS(reference); // Assuming static launching
                Actuation.turretMoveTowards(AutoLaunch.getTargetRot());
//                Actuation.setFlywheel(AutoLaunch.getTargetVel());
            }
        });
    }

    public static class Paths {
        public static class Blue {
            // Start
            public static Trajectory startLong = new Trajectory(FieldConstants.Start.blueLong);

            public static Trajectory startShort = new Trajectory(FieldConstants.Start.blueShort);
            // Spike 3
            public static Trajectory spike3 = new Trajectory()
                    .lineThrough(FieldConstants.Spike.Start.blue3, 0.5, 0.7)
                    .action(() -> Actuation.runIntake(true))
                    .action(() -> Actuation.runTransfer(true, false))
                    .lineTo(FieldConstants.Spike.End.blue3, 0.5, 0.7)
                    .action(() -> Actuation.runIntake(false))
                    .action(() -> Actuation.runTransfer(false, false));

            // Spike 2
            Trajectory spike2 = new Trajectory()
                    .lineThrough(FieldConstants.Spike.Start.blue2, 0.5, 0.7)
                    .action(() -> Actuation.runIntake(true))
                    .action(() -> Actuation.runTransfer(true, false))
                    .lineTo(FieldConstants.Spike.End.blue2, 0.5, 0.7)
                    .action(() -> Actuation.runIntake(false))
                    .action(() -> Actuation.runTransfer(false, false));


            // Spike 1
            Trajectory spike1 = new Trajectory()
                    .lineThrough(FieldConstants.Spike.Start.blue1, 0.5, 0.7)
                    .action(() -> Actuation.runIntake(true))
                    .action(() -> Actuation.runTransfer(true, false))
                    .lineTo(FieldConstants.Spike.End.blue1, 0.5, 0.7)
                    .action(() -> Actuation.runIntake(false))
                    .action(() -> Actuation.runTransfer(false, false));

            // Long Launch
            Trajectory launchLong = new Trajectory()
                    .lineTo(FieldConstants.Launch.blueLong);

            // Short Launch
            Trajectory launchShort = new Trajectory()
                    .lineTo(FieldConstants.Launch.blueShort);
            // Gate
            Trajectory gate = new Trajectory()
                    .lineTo(FieldConstants.Gate.blue);
        }

        public static class Red {

        }
    }
}