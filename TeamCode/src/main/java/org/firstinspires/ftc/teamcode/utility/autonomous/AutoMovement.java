package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;
import static org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore.getMove;
import static org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore.getStrafe;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.cameraVision.AprilTagDetection;
import org.firstinspires.ftc.teamcode.utility.cameraVision.ArtifactDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.List;

@Config
public class AutoMovement {

    //    public static void main(String[] args) {
//        System.out.println("Ryan Pergola mad cute :)");
//    }

    public static final double closestLaunchDist = 30; // way to many kewords man, consider changing to: double closestLaunchDIst
    public static boolean isTracking = false;
    public static boolean ready = false;

    public static boolean lockTracking = false;

    /**
     * Automatically intakes the closest artifact ahead of the robot
     */
    public static void autoIntakeArtifact() {
        boolean finished = false;
        while (!finished) {
            finished = ArtifactDetection.goToArtifact();
        }
    }

    /**
     * Finds the shortest pose to the long launch zone
     * @param team team color
     * Desmos Graph: <a href="https://www.desmos.com/calculator/rzvjeeb3be">...</a>
     */
    public static Pose closestLongLaunch(String team) {
        Pose pos = OttoCore.robotPose;
        Point goal;
        double x_closest, y_closest, heading;
        if (team.equalsIgnoreCase("red")) {
            // Long Red
            x_closest = (pos.x + pos.y - 49.0) / 2.0;
            // Clamp to make sure that the robot can launch from the position
            x_closest = Math.max(0, Math.min(x_closest, 72-closestLaunchDist));
            y_closest = x_closest + 49;
            goal = FieldConstants.Goal.red;
            heading = Math.atan2(y_closest - goal.y, x_closest - goal.x);
        } else {
            // Long Blue
            x_closest = (pos.x - pos.y - 49.0) / 2.0;
            // Clamp to make sure that the robot can launch from the position
            x_closest = Math.max(0, Math.min(x_closest, 72-closestLaunchDist));
            y_closest = -x_closest - 49;
            goal = FieldConstants.Goal.blue;
            heading = Math.atan2(y_closest - goal.y, x_closest - goal.x);
        }

        if (AutoLaunch.inLaunchZone() && !pos.withinRange(new Pose(goal.x, goal.y, heading), closestLaunchDist, closestLaunchDist, Math.toRadians(360))) {
            return pos;
        } else {
            return new Pose(x_closest, y_closest, heading);
        }
    }

    /**
     * Finds the shortest point to the short launch zone
     * @param team team color
     * Desmos Graph: <a href="https://www.desmos.com/calculator/rzvjeeb3be">...</a>
     */
    public static Pose closestShortLaunch(String team) {
        Pose pos = OttoCore.robotPose;
        Point goal;
        double x_closest, y_closest, heading;
        if (team.equalsIgnoreCase("red")) {
            // Short Red
            x_closest = (pos.x - pos.y) / 2.0;
            // Clamp to make sure that the robot can launch from the position
            x_closest = Math.max(-72, Math.min(x_closest, -49));
            y_closest = -x_closest;
            goal = FieldConstants.Goal.red;
            heading = Math.atan2(y_closest - goal.y, x_closest - goal.x);
        } else {
            // Long Blue
            x_closest = (pos.x + pos.y) / 2.0;
            // Clamp to make sure that the robot can launch from the position
            x_closest = Math.max(-72, Math.min(x_closest, -49));
            y_closest = (x_closest);
            goal = FieldConstants.Goal.blue;
            heading = Math.atan2(y_closest - goal.y, x_closest - goal.x);
        }

        if (AutoLaunch.inLaunchZone() && !pos.withinRange(new Pose(goal.x, goal.y, heading), closestLaunchDist, closestLaunchDist, Math.toRadians(360))) {
            return pos;
        } else {
            return new Pose(x_closest, y_closest, heading);
        }
    }

    /**
     * Creates a thread for operating the turret (auto-adjusting towards goal when close & maintaining flywheel velocity)
     * @param team team color
     * @return thread for turret operation
     */
    public static Thread turretOperation(String team) {
        return new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {

                List<LLResultTypes.FiducialResult> fids = AprilTagDetection.getFiducials();
                int goalID = team.equals("blue") ? 20 : 24;
                boolean trackingAprilTag = false;
                double tx = 0.0;

                //                Pose fiducialGlobalPos = new Pose(0, 0, 0);
                for (LLResultTypes.FiducialResult fid : fids) {
                    // Track AprilTag using center
                    if (fid.getFiducialId() == goalID) {
                        trackingAprilTag = true;
                        // Track using AprilTag data if the goal was detected
                        //                        fiducialGlobalPos.x = fid.getRobotPoseFieldSpace().getPosition().x;
                        //                        fiducialGlobalPos.y = fid.getRobotPoseFieldSpace().getPosition().y;
                        tx = Math.toRadians(fid.getTargetXDegrees());

                        //                        Point tag = goalID == 20 ? new Point(58, 54) : new Point(58, -54);
                        // Auto-adjusts robot position based on april tag angle;
                        //                        double global_angle = Actuation.getTurretGlobal()-tx;
                        //                        double m = Math.tan(global_angle), b = tag.y - tag.x*m;
                        //                        double new_x = (OttoCore.robotPose.x - m*b + m*OttoCore.robotPose.y) / (Math.pow(m, 2) + 1);
                        //                        OttoCore.robotPose = OttoCore.relativeTransform(new Pose(new_x, m*new_x+b, OttoCore.robotPose.heading), -ActuationConstants.Launcher.turretOffset, 0, 0);
                        //                        reference = OttoCore.relativeTransform(fiducialGlobalPos, ActuationConstants.Launcher.turretOffset, 0, 0);
                    }
                }

                //                AutoLaunch.updateAutoLaunchM(team, reference); // Assuming mobile or static launching
                AutoLaunch.updateAutoLaunchS(OttoCore.robotPose); // Assuming static launching

                if (AutoLaunch.closeToLaunchZone(20)) {
//                    if (trackingAprilTag) {
//                        Actuation.turretMoveTowards(Actuation.getTurretGlobal() - tx);
//                        ready = Math.abs(tx) < Math.toRadians(1);
//                    } else {
//                        Actuation.turretMoveTowards(AutoLaunch.getTargetRot());
//                        ready = Math.abs(AutoLaunch.getTargetRot() - Actuation.getTurretGlobal()) < Math.toRadians(1);
//                    }

                    Actuation.turretMoveTowards(AutoLaunch.getTargetRot());
                    ready = Math.abs(AutoLaunch.getTargetRot() - Actuation.getTurretGlobal()) < Math.toRadians(1);
                } else {
                    Actuation.turretMoveTowards(OttoCore.robotPose.heading);
                }

                Actuation.setFlywheel(AutoLaunch.getTargetVel());
                telemetry.addData("Target Flywheel Velocity", AutoLaunch.getTargetVel());
            }
        });
    }

    public static boolean isReady() {
        return ready;
    }
    public static void toggleTracking() {
        isTracking = !isTracking;
    }
    public static void setLockTracking(boolean lock) {
        lockTracking = lock;
    }
}