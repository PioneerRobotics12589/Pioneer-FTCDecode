package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.utility.Actuation.runIntake;
import static org.firstinspires.ftc.teamcode.utility.Actuation.runTransfer;
import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;
import static org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore.rotational;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.cameraVision.ArtifactDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Config
public class AutoMovement {

    //    public static void main(String[] args) {
//        System.out.println("Ryan Pergola mad cute :)");
//    }

    /**
     * Determines the robot angle, flywheel angular velocity, and the flywheel angle to launch from the current position
     * Desmos Graph: <a href="https://www.desmos.com/calculator/strglw28yh">...</a>
     * @param team team color
     */
    public static void autoLaunchStationary(String team, boolean shoot) {
        Point goal;
        if (team.equalsIgnoreCase("blue")) {
            goal = FieldConstants.Goal.blue;
        } else {
            goal = FieldConstants.Goal.red;
        }

        OttoCore.updatePosition();
        Pose position = new Pose(OttoCore.robotPose); // Get current position

        // Distance in inches
        double dist = Math.sqrt(Math.pow(position.x - goal.x, 2) + Math.pow(position.y - goal.y, 2));
        dist = dist / 39.37; // Convert from inches to meters

        final double g = -9.8; // Gravitational constant

        // Angle between robot and goal
        double angleRobot = Math.atan2(goal.y - position.y, goal.x - position.x);

        // Change in height from launcher to goal
        double height = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;

//        double flywheelAngle = 0.5*Math.atan(-dist/height) + Math.PI/2.0; // Optimal flywheel angle
        double angleFlywheel = Math.toRadians(55); // Angle of the flywheel

        // As seen in the Desmos graph, if the robot is to close, there is no possible way that it can be launched such that it will hit the target
        // to make sure that the program doesn't crash due to this issue in match, we need to make sure that everything under the square root is positive.
        double sqrt = g * Math.pow(dist, 2.0) / ((height - dist * Math.tan(angleFlywheel)) * (2.0 * Math.pow(Math.cos(angleFlywheel), 2.0)));

        double linVel = 0.0; // Linear velocity of the flywheel
        if (sqrt > 0) {
            linVel = Math.sqrt(sqrt);
        }

        // Convert from linear to angular velocity with scaling
        int flyVel = getFlyVel(linVel);

        telemetry.addData("Flywheel Velocity", flyVel);
        telemetry.addData("Robot angle", (angleRobot * Math.PI)/180.0);

        Actuation.setFlywheel(flyVel); // Speed up flywheel

        // Turn to goal
        OttoCore.updatePosition();
        Pose targetPose = new Pose(OttoCore.robotPose.x, OttoCore.robotPose.y, angleRobot);
        if (!(OttoCore.robotPose.withinRange(targetPose, 0.2, 0.2, Math.toRadians(2)))) {
            OttoCore.robotPose.heading = correctAngle(targetPose.heading, OttoCore.robotPose.heading);
            OttoCore.moveTowards(targetPose, 0.3, 0.3);
            OttoCore.updatePosition();
        }

        Actuation.runIntake(shoot);
        Actuation.runTransfer(shoot, true);
    }

    public static void autoLaunchStationary(String team) throws InterruptedException {
        Point goal;
        if (team.equalsIgnoreCase("blue")) {
            goal = FieldConstants.Goal.blue;
        } else {
            goal = FieldConstants.Goal.red;
        }

        OttoCore.updatePosition();
        Pose position = new Pose(OttoCore.robotPose); // Get current position

        // Distance in inches
        double dist = Math.sqrt(Math.pow(position.x - goal.x, 2) + Math.pow(position.y - goal.y, 2));
        dist = dist / 39.37; // Convert from inches to meters

        final double g = -9.8; // Gravitational constant

        // Angle between robot and goal
        double angleRobot = Math.atan2(goal.y - position.y, goal.x - position.x);

        // Change in height from launcher to goal
        double height = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;

//        double flywheelAngle = 0.5*Math.atan(-dist/height) + Math.PI/2.0; // Optimal flywheel angle
        double angleFlywheel = Math.toRadians(55); // Angle of the flywheel

        // As seen in the Desmos graph, if the robot is to close, there is no possible way that it can be launched such that it will hit the target
        // to make sure that the program doesn't crash due to this issue in match, we need to make sure that everything under the square root is positive.
        double sqrt = g * Math.pow(dist, 2.0) / ((height - dist * Math.tan(angleFlywheel)) * (2.0 * Math.pow(Math.cos(angleFlywheel), 2.0)));

        double linVel = 0.0; // Linear velocity of the flywheel
        if (sqrt > 0) {
            linVel = Math.sqrt(sqrt);
        }

        // Convert from linear to angular velocity with scaling
        int flyVel = getFlyVel(linVel);

        telemetry.addData("Flywheel Velocity", flyVel);
        telemetry.addData("Robot angle", (angleRobot * Math.PI)/180.0);

        Actuation.setFlywheel(flyVel); // Speed up flywheel

        // Turn to goal
        OttoCore.updatePosition();
        Pose targetPose = new Pose(OttoCore.robotPose.x, OttoCore.robotPose.y, angleRobot);
        while (!(OttoCore.robotPose.withinRange(targetPose, 10, 10, Math.toRadians(2)))) {
            OttoCore.robotPose.heading = correctAngle(targetPose.heading, OttoCore.robotPose.heading);
            OttoCore.moveTowards(targetPose, 0.5, 0.8);
            OttoCore.updatePosition();
        }

        launch();
    }

    public static double goalRotation(Pose targetPose, Point goal) {
        return Math.atan2(goal.y - targetPose.y, goal.x - targetPose.x);
    }
    public static void autoFlywheelVel(Pose targetPose, Point goal) {

        // Distance in inches
        double dist = Math.sqrt(Math.pow(targetPose.x - goal.x, 2) + Math.pow(targetPose.y - goal.y, 2));
        dist = dist / 39.37; // Convert from inches to meters

        final double g = -9.8; // Gravitational constant

        // Change in height from launcher to goal
        double height = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;

//        double flywheelAngle = 0.5*Math.atan(-dist/height) + Math.PI/2.0; // Optimal flywheel angle
        double angleFlywheel = Math.toRadians(55); // Angle of the flywheel

        // As seen in the Desmos graph, if the robot is to close, there is no possible way that it can be launched such that it will hit the target
        // to make sure that the program doesn't crash due to this issue in match, we need to make sure that everything under the square root is positive.
        double sqrt = g * Math.pow(dist, 2.0) / ((height - dist * Math.tan(angleFlywheel)) * (2.0 * Math.pow(Math.cos(angleFlywheel), 2.0)));

        double linVel = 0.0; // Linear velocity of the flywheel
        if (sqrt > 0) {
            linVel = Math.sqrt(sqrt);
        }

        // Convert from linear to angular velocity with scaling
        int flyVel = getFlyVel(linVel);

        Actuation.setFlywheel(flyVel); // Speed up flywheel
    }

    /**
     * Determines the robot angle and flywheel velocity to launch whilst moving towards a specified pose
     * Desmos Graph: <a href="https://www.desmos.com/3d/vlh4zicvzv">...</a>
     *
     * @param team team color
     */
    public static void autoLaunchMoving(String team, Pose targetPose) {
        Point goal;
        if (team.equalsIgnoreCase("blue")) {
            goal = FieldConstants.Goal.blue;
        } else {
            goal = FieldConstants.Goal.red;
        }

        OttoCore.updatePosition();
        Pose position = new Pose(OttoCore.robotPose); // Get current position

        double d_x = (goal.x - position.x) / 39.37; // X distance
        double d_y = (goal.y - position.y) / 39.37; // Y distance
        double d_tot = (Math.sqrt(Math.pow(d_x, 2) + Math.pow(d_y, 2))); // Total distance
        // Change in height
        double height = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;

        Pose velocities = OttoCore.getVelocity(); // Get current velocities
        double v_p = velocities.x * Math.cos(position.heading) + velocities.y * Math.sin(position.heading); // Velocity Parallel to heading

        final double g = -9.8; // Gravitational Constant

        double theta_f = Math.toRadians(55); // Flywheel Angle

        double theta_r = Math.atan2(d_y, d_x); // Robot angle with first guess

        double v_f = 0.0; // Flywheel Speed (linear velocity)

        // Iterate to diverge robot angle
        for (int i = 0; i < 10; i++) {
            // As seen in the Desmos graph, if the robot is to close, there is no possible way that it can be launched such that it will hit the target
            // to make sure that the program doesn't crash due to this issue in match, we need to make sure that everything under the square root is positive.
            double sqrt = -Math.pow(d_tot, 2) * (2 * d_tot * g * Math.sin(theta_f) * Math.cos(theta_f) - 2 * g * height * Math.pow(Math.cos(theta_f), 2) - Math.pow(v_p, 2) * Math.pow(Math.sin(theta_f), 2));
            if (sqrt < 0) {
                break;
            }

            // Initial Flywheel Velocity - Formula found in linked Desmos graph (THIS FORMULA IS PAST HUMAN COMPREHENSION AT FACE VALUE)
            v_f = (1.0 / Math.cos(theta_f) * (Math.sqrt(sqrt) - d_tot * v_p * Math.sin(theta_f) + 2.0 * height * v_p * Math.cos(theta_f))) / (2 * (d_tot * Math.sin(theta_f) - height * Math.cos(theta_f)));

            // Total time that the artifact is a projectile
            double t_proj = (-v_f * Math.sin(theta_f) - Math.sqrt(Math.pow(v_f * Math.sin(theta_f), 2) - 2 * g * height)) / g;

            // Update robot heading
            theta_r = Math.atan2(d_y - velocities.y * t_proj, d_x - velocities.x * t_proj);
        }

        // Convert linear flywheel velocity to angular
        int w_f = getFlyVel(v_f);

        // Turn to goal
        OttoCore.updatePosition();
        Pose tp = new Pose(targetPose.x, targetPose.y, theta_r); // Target position

        telemetry.addData("Flywheel Velocity", w_f);

        if (!(OttoCore.robotPose.withinRange(tp, 0.2, 0.2, Math.toRadians(2)))) {
            OttoCore.robotPose.heading = correctAngle(tp.heading, OttoCore.robotPose.heading);
            OttoCore.moveTowards(targetPose, 0.3, 0.3);
            OttoCore.updatePosition();

            // Speed up flywheel
            Actuation.setFlywheel(w_f);
            if (inLaunchZone()) {
                // Launch Artifacts
                Actuation.runTransfer(true, true);
                Actuation.runIntake(true);
            } else {
                Actuation.runTransfer(false, true);
                Actuation.runIntake(false);
            }
        }

        // Stop flywheel & transfer of artifacts
        Actuation.setFlywheel(0);
        Actuation.runTransfer(false, false);
        Actuation.runIntake(false);
    }

    /**
     * Determines the robot angle and flywheel velocity to launch whilst moving in TeleOp
     * Desmos Graph: <a href="https://www.desmos.com/3d/vlh4zicvzv">...</a>
     *
     * @param team team color
     * @param move move speed
     * @param strafe strafe speed
     */
    public static void autoLaunchMoving(String team, double move, double strafe) {
        Point goal;
        if (team.equalsIgnoreCase("blue")) {
            goal = FieldConstants.Goal.blue;
        } else {
            goal = FieldConstants.Goal.red;
        }

        OttoCore.updatePosition();
        Pose position = new Pose(OttoCore.robotPose); // Get current position

        double d_x = (goal.x - position.x) / 39.37; // X distance
        double d_y = (goal.y - position.y) / 39.37; // Y distance
        double d_tot = (Math.sqrt(Math.pow(d_x, 2) + Math.pow(d_y, 2))); // Total distance
        // Change in height
        double height = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;

        Pose velocities = OttoCore.getVelocity(); // Get current velocities
        double v_p = velocities.x * Math.cos(position.heading) + velocities.y * Math.sin(position.heading); // Velocity Parallel to heading

        final double g = -9.8; // Gravitational Constant

        double theta_f = Math.toRadians(55); // Flywheel Angle

        double theta_r = Math.atan2(d_y, d_x); // Robot angle with first guess

        double v_f = 0.0; // Flywheel Speed (linear velocity)

        // Iterate to diverge robot angle
        for (int i = 0; i < 10; i++) {
            // As seen in the Desmos graph, if the robot is to close, there is no possible way that it can be launched such that it will hit the target
            // to make sure that the program doesn't crash due to this issue in match, we need to make sure that everything under the square root is positive.
            double sqrt = -Math.pow(d_tot, 2) * (2 * d_tot * g * Math.sin(theta_f) * Math.cos(theta_f) - 2 * g * height * Math.pow(Math.cos(theta_f), 2) - Math.pow(v_p, 2) * Math.pow(Math.sin(theta_f), 2));
            if (sqrt < 0) {
                break;
            }

            // Initial Flywheel Velocity - Formula found in linked Desmos graph (THIS FORMULA IS PAST HUMAN COMPREHENSION AT FACE VALUE)
            v_f = (1.0 / Math.cos(theta_f) * (Math.sqrt(sqrt) - d_tot * v_p * Math.sin(theta_f) + 2.0 * height * v_p * Math.cos(theta_f))) / (2 * (d_tot * Math.sin(theta_f) - height * Math.cos(theta_f)));

            // Total time that the artifact is a projectile
            double t_proj = (-v_f * Math.sin(theta_f) - Math.sqrt(Math.pow(v_f * Math.sin(theta_f), 2) - 2 * g * height)) / g;

            // Update robot heading
            theta_r = Math.atan2(d_y - velocities.y * t_proj, d_x - velocities.x * t_proj);
        }

        // Convert linear flywheel velocity to angular
        int w_f = getFlyVel(v_f);

        // Drive & Turn
        telemetry.addData("theta_r", theta_r);
        telemetry.addData("d_x to goal", d_x);
        telemetry.addData("d_y to goal", d_y);
        telemetry.addData("w_f", w_f);

        if (Math.abs(OttoCore.robotPose.heading - theta_r) > Math.toRadians(2)) {
            // Corrects robot heading to take the shortest angle
            OttoCore.robotPose.heading = correctAngle(theta_r, OttoCore.robotPose.heading);

            double rotSignal = rotational.calculateSignal(theta_r, OttoCore.robotPose.heading);
            double clampedRot = Math.max(-1, Math.min(1, rotSignal));

            Actuation.drive(move, clampedRot, strafe);
            telemetry.addData("Rotation signal", clampedRot);
        }

//        Actuation.setFlywheel(w_f);

//        // Launch artifacts if it is inside the launch zone
//        if (inLaunchZone()) {
//            Actuation.runTransfer(true, true);
//            Actuation.runIntake(true);
//        } else {
//            Actuation.runTransfer(false, true);
//            Actuation.runIntake(false);
//        }
    }

    /**
     * Scales the flywheel velocity based off testing
     * @param linVel calculated linear flywheel velocity
     * @return scaled flywheel angular velocity
     */
    private static int getFlyVel(double linVel) {
        int flyVel = (int) (linVel / (ActuationConstants.Drivetrain.flwheelRad + ActuationConstants.Launcher.artifactRadius) * 180.0 / Math.PI);

        // Due to inaccuracies that would be too difficult to account for, such as inconsistent actual launch angle, drag, and spin
        // we use a linear scale to roughly account for these inconsistencies.
        double kMult = 0.655033; // Flywheel tunable multiplier
        double kBias = 674.65202; // Flywheel tunable bias

        flyVel = (int) (flyVel * kMult + kBias);
        return flyVel;
    }

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
        double turnRate;
        do {
            turnRate = ArtifactDetection.trackArtifact(color);
            Actuation.drive(0, turnRate, 0);
        } while (Math.abs(turnRate) > 0.1);
    }

    /**
     * Autonomously takes the shortest path to the long-launch zone
     * Desmos Graph: <a href="https://www.desmos.com/calculator/rzvjeeb3be">...</a>
     */
    public static Pose closestLongLaunch() {
        Pose curr = new Pose(OttoCore.robotPose);
        Pose closest = new Pose(0, 0, 0);

        if (inLaunchZone()) {
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

        return closest;
    }

    /**
     * Autonomously takes the shortest path to the short-launch zone
     */
    public static Pose closestShortLaunch() {
        Pose curr = new Pose(OttoCore.robotPose);
        Pose closest = new Pose(0, 0, 0);

        if (inLaunchZone()) {
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

        closest.x = Math.max(minX, Math.min(maxX, closest.x));

        closest.y = Math.abs(closest.x) - 1.25;

        return closest;
    }

    /**
     * Launches artifacts
     */
    public static void launch() throws InterruptedException {
        sleep(1500);

        // Push artifacts into flywheel
        Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed*0.8);
        Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed*0.8);

        // Launch for 1.5s
        sleep(2000);

        Actuation.setFlywheel(0);
        Actuation.runIntake(false);
        Actuation.runTransfer(false, true);
    }

    /**
     * Dear Ryan Reynolds, please forgive me, I am ashamed of this function.
     * @param distance distance to be moved forward
     */
    public static void moveForward(double distance) {
        Pose pos = new Pose(OttoCore.robotPose);
        Pose newPose = new Pose(pos.x+distance*Math.cos(pos.heading), pos.y+distance*Math.sin(pos.heading), pos.heading);
        Trajectory path = new Trajectory()
                .lineTo(newPose);
        path.run();
    }

    /**
     * Corrects the current heading to make sure that it takes the shortest angle to turn
     * @param target target angle
     * @param current current angle
     * @return new current angle
     */
    public static double correctAngle(double target, double current) {
        double newCurrent = current;
        if (target > current) {
            while (Math.abs(target - newCurrent) > Math.toRadians(180)) {
                newCurrent += 2 * Math.PI;
            }
        } else if (target < current) {
            while (Math.abs(target - newCurrent) > Math.toRadians(180)) {
                newCurrent -= 2 * Math.PI;
            }
        }
        return newCurrent;
    }

    /**
     * Determines whether the robot is inside a launch zone
     * @return true: robot is in a launch zone; false: robot is NOT in a launch zone
     */
    public static boolean inLaunchZone() {
        Pose pos = OttoCore.robotPose;
        // X-Bounds for long-launch: -24 to 24
        // X-Bounds for short-launch: None
        // Y-Bounds for long-launch: [-|x|-48, -72]
        // Y-Bounds for short-launch: [|x|-1.25, 72]
        return (pos.x >= -24 && pos.x <= 24 && pos.y <= -Math.abs(pos.x) - 48) || (pos.y >= Math.abs(pos.x) - 1.25);
    }
}