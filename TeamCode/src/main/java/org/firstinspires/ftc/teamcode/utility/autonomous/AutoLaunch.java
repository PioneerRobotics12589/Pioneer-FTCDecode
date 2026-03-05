package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

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

    public static void setTargetRot(double angle) {
        targetRot = angle;
    }

    public static void launchOperation() {
        while (!(inLaunchZone() && notTooClose() && AutoMovement.readyToLaunch())) {
            Actuation.runIntake(false);
            Actuation.runTransfer(false);
        }
    }

    /**
     * Updates the target values for auto-launching while moving
     */
    public static void updateAutoLaunchMobile(Pose reference) {
        Point goal = team.equalsIgnoreCase("blue") ? FieldConstants.Goal.blue : FieldConstants.Goal.red;

        double d_x = (reference.x - goal.x) / 39.37; // X distance
        double d_y = (reference.y - goal.y) / 39.37; // Y distance

        Pose vel = OttoCore.getVelocity(); // Get current velocities

        targetVel = getFlyVel((4.8441270688 + (0.1802078198 * d_x) + (0.1833721958 * d_y) + (0.0413811466 * vel.x) + (0.0279143290 * vel.y) + (0.1326523733 * d_x*d_x) + (-0.0507610667 * d_x*d_y) + (-0.2052237052 * d_x*vel.x) + (-0.0243015090 * d_x*vel.y) + (0.1328960696 * d_y*d_y) + (-0.0271567227 * d_y*vel.x) + (-0.2095355507 * d_y*vel.y) + (0.0676393013 * vel.x*vel.x) + (0.0443569373 * vel.x*vel.y) + (0.0769738471 * vel.y*vel.y) + (-0.0098544120 * d_x*d_x*d_x) + (-0.0027375016 * d_x*d_x*d_y) + (0.0113618273 * d_x*d_x*vel.x) + (0.0053323178 * d_x*d_x*vel.y) + (-0.0032396893 * d_x*d_y*d_y) + (0.0090960928 * d_x*d_y*vel.x) + (0.0096150530 * d_x*d_y*vel.y) + (0.0012637035 * d_x*vel.x*vel.x) + (-0.0071646824 * d_x*vel.x*vel.y) + (-0.0034042167 * d_x*vel.y*vel.y) + (-0.0098325331 * d_y*d_y*d_y) + (0.0054107393 * d_y*d_y*vel.x) + (0.0125597581 * d_y*d_y*vel.y) + (-0.0021258349 * d_y*vel.x*vel.x) + (-0.0073579327 * d_y*vel.x*vel.y) + (-0.0007484920 * d_y*vel.y*vel.y) + (-0.0032378693 * vel.x*vel.x*vel.x) + (0.0003376211 * vel.x*vel.x*vel.y) + (0.0011173520 * vel.x*vel.y*vel.y) + (-0.0028310896 * vel.y*vel.y*vel.y)));
        targetRot = getRot(0.7965269017 + (-0.5502373893 * d_x) + (0.5382293956 * d_y) + (0.2302603267 * vel.x) + (-0.2217951683 * vel.y) + (0.0948967216 * d_x*d_x) + (0.0038175622 * d_x*d_y) + (-0.0691073204 * d_x*vel.x) + (-0.0439749237 * d_x*vel.y) + (-0.0949920764 * d_y*d_y) + (0.0408508871 * d_y*vel.x) + (0.0691250905 * d_y*vel.y) + (0.0048890543 * vel.x*vel.x) + (0.0013956383 * vel.x*vel.y) + (-0.0074973437 * vel.y*vel.y) + (-0.0035247029 * d_x*d_x*d_x) + (-0.0109953764 * d_x*d_x*d_y) + (0.0024813607 * d_x*d_x*vel.x) + (0.0076420578 * d_x*d_x*vel.y) + (0.0104375821 * d_x*d_y*d_y) + (0.0061484484 * d_x*d_y*vel.x) + (-0.0060848250 * d_x*d_y*vel.y) + (0.0002822522 * d_x*vel.x*vel.x) + (-0.0036288430 * d_x*vel.x*vel.y) + (0.0000457961 * d_x*vel.y*vel.y) + (0.0037589706 * d_y*d_y*d_y) + (-0.0075781053 * d_y*d_y*vel.x) + (-0.0026426180 * d_y*d_y*vel.y) + (0.0006803577 * d_y*vel.x*vel.x) + (0.0039938061 * d_y*vel.x*vel.y) + (-0.0002690869 * d_y*vel.y*vel.y) + (-0.0000206521 * vel.x*vel.x*vel.x) + (-0.0011237318 * vel.x*vel.x*vel.y) + (0.0001889585 * vel.x*vel.y*vel.y) + (0.0002403688 * vel.y*vel.y*vel.y), Math.atan2(d_y, d_x));
        // Convert linear flywheel velocity to angular
        telemetry.addData("Target Flywheel Velocity", targetVel);
        telemetry.addData("Target Robot Angle", targetRot);
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
        double kMult = 0.481013; // Flywheel tunable multiplier
        double kBias = 877.29114; // Flywheel tunable bias

        flyVel = (int) (flyVel * kMult + kBias);
        return flyVel;
    }

    private static double getRot(double rot, double directRot) {
        double difference = rot-directRot;

        double kMult = 1.0;
        double kBias = 0.0;

        return rot + difference*kMult + kBias;
    }

    /**
     * Determines whether the robot is inside a launch zone
     * @return true: robot is in a launch zone; false: robot is NOT in a launch zone
     */
    public static boolean inLaunchZone() {
        Pose pos = OttoCore.robotPose;
        if (pos.x >= 0 && pos.x <= 72 && pos.y >= -pos.x && pos.y <= pos.x) {
            // In short launch zone
            return true;
        }
        // In long launch zone
        return pos.x >= -72 && pos.x <= -49 && pos.y >= pos.x + 49 && pos.y <= -pos.x - 49;

        // X-Bounds for long-launch: -72 to -49
        // X-Bounds for short-launch: 0 to 72
        // Y-Bounds for long-launch: between y = x + 49 and y = -x - 49
        // Y-Bounds for short-launch: between y = x and y = -x
    }

    public static boolean notTooClose() {
        Pose pos = OttoCore.robotPose;
        Point goal = team.equalsIgnoreCase("blue") ? FieldConstants.Goal.blue : FieldConstants.Goal.red;
        return ((Math.sqrt((goal.x-pos.x)*(goal.x-pos.x) + (goal.y-pos.y)*(goal.y-pos.y))) > 55);
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
