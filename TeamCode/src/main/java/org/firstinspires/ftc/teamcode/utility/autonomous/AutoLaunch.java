package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

public class AutoLaunch {
    private static int targetVel;
    private static double targetRot;
    private static String team;

    /**
     * Sets the team color to know the goal's position
     * @param teamColor team color ("red" or "blue")
     */
    public static void setTeam(String teamColor) {
        team = teamColor;
    }

    /**
     * Gets the target flywheel angular velocity
     * @return flywheel angular velocity
     */
    public static int getTargetVel() {
        return targetVel;
    }

    /**
     * Gets the target global heading for the launcher
     * @return angle in radians
     */
    public static double getTargetRot() {
        return targetRot;
    }

    /**
     * Updates the target values for auto-launching while moving
     */
    public static void updateAutoLaunchMobile(Pose reference) {
        Point goal = team.equalsIgnoreCase("blue") ? FieldConstants.Goal.blue : FieldConstants.Goal.red;

        double t_future = 0.0;

        Pose vel = OttoCore.getVelocity(); // Get current velocities

        double vx = vel.x / 39.37;
        double vy = vel.y / 39.37;
        double dx = (goal.x - reference.x - vel.x * t_future) / 39.37; // X distance
        double dy = (goal.y - reference.y - vel.y * t_future) / 39.37; // Y distance

        final double h = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;
        final double g = -9.8;
        final double theta_f = 50.0 * Math.PI / 180.0; // Flywheel Angle
        final double lr = 0.25; // Learning Rate

        int iterations = 1;

        double curr_t = 0.67; // Initial projectile time "guess" (Keep at 0.67)
        double prev_t = 0.0; // Used to measure convergence
        double curr_v = 0.0;

        class Iterate {
            double velocity(double t_proj, double vel) {
                double sqrt = ((dx - vx*t_proj)*(dx - vx*t_proj) + (dy - vy*t_proj)*(dy - vy*t_proj)) / (t_proj*t_proj*Math.cos(theta_f)*Math.cos(theta_f));
                // Catch domain error
                if (sqrt < 0) {
                    return vel;
                }
                return Math.sqrt(sqrt);
            }

            double heading(double t_proj) {
                return Math.atan2(dy - vy*t_proj, dx - vx*t_proj);
            }

            double time(double t_proj, double vel) {
                double sqrt = (vel*vel*Math.sin(theta_f)*Math.sin(theta_f)) + 2*g*h;
                // Catch domain error
                if (sqrt < 0) {
                    return t_proj;
                }
                return t_proj + lr * (((-vel*Math.sin(theta_f) - Math.sqrt(sqrt)) / g) - t_proj);
            }
        }

        Iterate iter = new Iterate();

        curr_v = iter.velocity(curr_t, curr_v);

        // Perform Iterations
        while (Math.abs(curr_t - prev_t) > 0.000001 && iterations < 200) {
            iterations++;

            prev_t = curr_t;
            curr_t = iter.time(curr_t, curr_v);
            curr_v = iter.velocity(curr_t, curr_v);
        }

        targetRot = ActuationConstants.Launcher.sotmAdjustMult*(iter.heading(curr_t) - Math.atan2(dy, dx)) + Math.atan2(dy, dx);
        targetVel = getFlyVel(curr_v);

        // Convert linear flywheel velocity to angular
        telemetry.addData("Target Flywheel Velocity", targetVel);
        telemetry.addData("Target Turret Angle", targetRot);
        telemetry.addData("Current Turret Angle", AngleUnit.normalizeRadians(Actuation.getTurretGlobal()));
    }

    /**
     * Updates the target values for auto-launching while stationary
     * Desmos Graph: <a href="https://www.desmos.com/calculator/strglw28yh">...</a>
     */
    public static void updateAutoLaunchStatic(Pose reference) {
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
        double angleFlywheel = Math.toRadians(50); // Angle of the flywheel

        // As seen in the Desmos graph, if the robot is to close, there is no possible way that it can be launched such that it will hit the target
        // to make sure that the program doesn't crash due to this issue in match, we need to make sure that everything under the square root is nonzero and positive.
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
        // Tune based on short/long launch coefficients
        double c1 = -2.13833 * Math.pow(10, -8);
        double c2 = 0.000111738;
        double c3 = -0.215125;
        double c4 = 181.50463;
        double c5 = -55735.9109;

        flyVel = (int) (c1*Math.pow(flyVel, 4) + c2*Math.pow(flyVel, 3) + c3*Math.pow(flyVel, 2) + c4*flyVel + c5);
        return flyVel;
    }

    /**
     * Determines whether the robot is inside a launch zone
     * @return true: robot is in a launch zone; false: robot is NOT in a launch zone
     */
    // i love dcosig
    public static boolean inLaunchZone() {
        Pose pos = OttoCore.robotPose;

        // Half length && half width
        double len = 8.5, wid = 6.25;

        class subMethods {
            boolean pointInZone(Point p) {
                if (p.x >= 0 && p.x <= 72 && p.y >= -p.x && p.y <= p.x) {
                    // In short launch zone
                    return true;
                }
                // In long launch zone
                return p.x >= -72 && p.x <= -49 && p.y >= p.x + 49 && p.y <= -p.x - 49;
                // X-Bounds for long-launch: -72 to -49
                // X-Bounds for short-launch: 0 to 72
                // Y-Bounds for long-launch: between y = x + 49 and y = -x - 49
                // Y-Bounds for short-launch: between y = x and y = -x
            }

            Point rotPoint(Pose pos, double x, double y) {
                return new Point(x*Math.cos(pos.heading) - y*Math.sin(pos.heading) + pos.x, x*Math.sin(pos.heading) + y*Math.cos(pos.heading) + pos.y);
            }
        }

        subMethods subM = new subMethods();

        // Robot corners
        Point c1 = subM.rotPoint(pos, len, wid);
        Point c2 = subM.rotPoint(pos, -len, wid);
        Point c3 = subM.rotPoint(pos, -len, -wid);
        Point c4 = subM.rotPoint(pos, len, -wid);

        return (subM.pointInZone(c1) || subM.pointInZone(c2) || subM.pointInZone(c3) || subM.pointInZone(c4));
    }

    public static boolean notTooClose() {
        Pose pos = OttoCore.robotPose;
        Point goal = team.equalsIgnoreCase("blue") ? FieldConstants.Goal.blue : FieldConstants.Goal.red;
        return ((Math.sqrt((goal.x-pos.x)*(goal.x-pos.x) + (goal.y-pos.y)*(goal.y-pos.y))) > 50);
    }

    /**
     * Determines whether the robot is close to the launch zone
     * @return true: robot is
     */
    public static boolean closeToLaunchZone(double maxDist) {
        if (inLaunchZone()) {
            return true;
        }

        Pose pos = OttoCore.robotPose;
        if (pos.x >= -maxDist && pos.x <= 72 && pos.y >= -pos.x-maxDist && pos.y <= pos.x+maxDist) {
            // In short launch zone
            return true;
        }
        // In long launch zone
        return pos.x >= -72 && pos.x <= -49 + maxDist && pos.y >= pos.x + 49 - maxDist && pos.y <= -pos.x - 49 + maxDist;
    }

    public static boolean closeToShort(double maxDist) {
        Pose pos = OttoCore.robotPose;
        return pos.x >= -maxDist && pos.x <= 72 && pos.y >= -pos.x-maxDist && pos.y <= pos.x+maxDist;
    }

    public static boolean closeToLong(double maxDist) {
        Pose pos = OttoCore.robotPose;
        return pos.x >= -72 && pos.x <= -49 + maxDist && pos.y >= pos.x + 49 - maxDist && pos.y <= -pos.x - 49 + maxDist;
    }
}
