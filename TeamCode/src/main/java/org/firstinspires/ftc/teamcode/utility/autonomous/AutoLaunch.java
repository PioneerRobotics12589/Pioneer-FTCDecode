package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class AutoLaunch {
    private static int targetVel;
    private static double targetRot;
    private static String team;

    public static void setTeam(String teamColor) {
        team = teamColor;
    }

    public static int getTargetVel() {
        return targetVel;
    }

    public static double getTargetRot() {
        return targetRot;
    }

    /**
     * Updates the target values for auto-launching while moving
     */
    public static void updateAutoLaunchM(Pose reference) {
        Point goal = team.equalsIgnoreCase("blue") ? FieldConstants.Goal.blue : FieldConstants.Goal.red;

        double d_x = (goal.x - reference.x) / 39.37; // X distance
        double d_y = (goal.y - reference.y) / 39.37; // Y distance
        double d_tot = (Math.sqrt(Math.pow(d_x, 2) + Math.pow(d_y, 2))); // Total distance
        // Change in height
        double height = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;

        Pose velocities = OttoCore.getVelocity(); // Get current velocities
        double v_p = velocities.x * Math.cos(reference.heading) + velocities.y * Math.sin(reference.heading); // Velocity Parallel to heading

        final double g = -9.8; // Gravitational Constant

        double theta_f = Math.toRadians(55); // Flywheel Angle

        targetRot = Math.atan2(d_y, d_x); // Robot angle with first guess

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
            targetRot = Math.atan2(d_y - velocities.y * t_proj, d_x - velocities.x * t_proj);
        }

        // Convert linear flywheel velocity to angular
        targetVel = getFlyVel(v_f);

        telemetry.addData("Target Flywheel Velocity", targetVel);
        telemetry.addData("Target Robot Angle", targetRot);
    }

    /**
     * Updates the target values for auto-launching while stationary
     * Desmos Graph: <a href="https://www.desmos.com/calculator/strglw28yh">...</a>
     */
    public static void updateAutoLaunchS(Pose reference) {
        Point goal = team.equalsIgnoreCase("blue") ? FieldConstants.Goal.blue : FieldConstants.Goal.red;

        // Distance in inches
        double dist = Math.sqrt(Math.pow(reference.x - goal.x, 2) + Math.pow(reference.y - goal.y, 2));
        dist = dist / 39.37; // Convert from inches to meters

        final double g = -9.8; // Gravitational constant ( as indicated by using the keyword final)

        // Angle between robot and goal
        targetRot = Math.atan2(goal.y - reference.y, goal.x - reference.x);

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
        targetVel = getFlyVel(linVel);
    }

    /**
     * Rotates to the angle in order to launch into the goal
     */
    public static void rotate() {
        rotate(0, 0);
    }

    /**
     * Rotates to the angle in order to launch into the goal while moving
     */
    public static void rotate(double move, double strafe) {
        if (Math.abs(OttoCore.robotPose.heading - targetRot) > Math.toRadians(0.5)) {
            double turnSig = OttoCore.getTurn(new Pose(OttoCore.robotPose.x, OttoCore.robotPose.y, targetRot), 0.8);
            Actuation.drive(move, turnSig, strafe);
        }
    }

    /**
     * Sets the flywheel speed to launch into the goal
     */
    public static void setFlywheel() {
        Actuation.setFlywheel(getTargetVel());
    }

    /**
     * Linear to angular flywheel velocity transformation and scaling based off testing
     * @param linVel calculated linear flywheel velocity
     * @return scaled flywheel angular velocity
     */
    private static int getFlyVel(double linVel) {
        int flyVel = (int) (linVel / (ActuationConstants.Drivetrain.flwheelRad + ActuationConstants.Launcher.artifactRadius) * 180.0 / Math.PI);

        // Due to inaccuracies that would be too difficult to account for, such as inconsistent actual launch angle, drag, and spin
        // we use a linear scale to roughly account for these inconsistencies.
        double kMult = 0.868718; // Flywheel tunable multiplier
        double kBias = 294.87254; // Flywheel tunable bias

        flyVel = (int) (flyVel * kMult + kBias);
        return flyVel;
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
