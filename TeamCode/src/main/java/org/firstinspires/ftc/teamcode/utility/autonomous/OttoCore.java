package org.firstinspires.ftc.teamcode.utility.autonomous;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.cameraVision.AprilTagDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PIDController;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.localization.IMUControl;
import org.firstinspires.ftc.teamcode.utility.localization.PinpointControl;

import java.util.List;
public class OttoCore {
    static final int SIDE_LENGTH = 6;

    public static Pose robotPose, lastPose;
    public static double ticks_left = 0, ticks_right = 0, ticks_back = 0;

    static double dx, dy, dtheta;
    static double dx_center, dx_perpendicular;
    static double prev_ticks_left, prev_ticks_right, prev_ticks_back;

//    static List<LynxModule> allHubs;
    public static VoltageSensor voltageSensor;

    static long lastTime;

    /**
     * Necessary in order to correctly initialize reading from odometry and accessing voltage sensor
     * @param hardwareMap Current hardware map
     */
    public static void setup(HardwareMap hardwareMap) {
        robotPose = new Pose(0, 0, 0);
        PinpointControl.setup(hardwareMap, ActuationConstants.Drivetrain.xOdoOffset, ActuationConstants.Drivetrain.yOdoOffset);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        robotPose = new Pose(0, 0, 0);
    }

    public static void setPose(Pose newPose) {
        PinpointControl.setPose(newPose);
        OttoCore.robotPose = newPose;
    }

    /**
     * Updates the robot's pose based off of encoder values from odometry
     */
    // All equations derived from and explained by:
    // https://gm0.org/en/latest/docs/software/concepts/odometry.html
    public static void updatePosition() {
//        for (LynxModule module : allHubs) {
//            module.clearBulkCache();
//        }
//
//        lastPose = new Pose(OttoCore.robotPose);
//        lastTime = System.nanoTime();
//
//        ticks_left = -Actuation.frontLeft.getCurrentPosition();
//        ticks_right = Actuation.frontRight.getCurrentPosition();
//        ticks_back = Actuation.backRight.getCurrentPosition();
//
//        double delta_ticks_left = (ticks_left - prev_ticks_left);
//        double delta_ticks_right = (ticks_right - prev_ticks_right);
//        double delta_ticks_back = (ticks_back - prev_ticks_back);
//
//        double inchesPerTick = ActuationConstants.Drivetrain.wheel_circ / ActuationConstants.Drivetrain.ticksPerRev;
//
//        // We divide each differential by ticks per revolution and multiply by the wheel circumference in order to account for real-world distance
//        delta_ticks_left *= inchesPerTick;
//        delta_ticks_right *= inchesPerTick;
//        delta_ticks_back *= inchesPerTick;
//
//        // Change in angle
//        double delta_theta = ((delta_ticks_left - delta_ticks_right) / ActuationConstants.Drivetrain.track_width);
//
//        // Change in the center position of the robot relative to itself (just the average of the parallel wheel diffs)
//        // aka Vertical displacement
//        double delta_center = ((delta_ticks_left + delta_ticks_right) / 2);
//
//        // Change in the perpendicular position of the robot relative to itself
//        // aka Horizontal displacement
//        double delta_perp = delta_ticks_back - (ActuationConstants.Drivetrain.forward_offset * delta_theta); // The other formula is negative of this
//
//        double dx = 0;
//        double dy = 0;
//        if (delta_theta != 0) { // Accounting for division by zero
//            dx = (delta_center * (Math.cos(robotPose.heading) * Math.sin(delta_theta) - Math.sin(robotPose.heading) * (1 - Math.cos(delta_theta)))
//                    + delta_perp * (Math.cos(robotPose.heading) * (Math.cos(delta_theta) - 1) - Math.sin(robotPose.heading) * Math.sin(delta_theta))) / delta_theta;
//
//            dy = (delta_center * (Math.sin(robotPose.heading) * Math.sin(delta_theta) + Math.cos(robotPose.heading) * (1 - Math.cos(delta_theta)))
//                    + delta_perp * (Math.sin(robotPose.heading) * (Math.cos(delta_theta) - 1) + Math.cos(robotPose.heading) * Math.sin(delta_theta))) / delta_theta;
//        }
//        else { // If delta_theta is 0 we use Euler Integration instead of Pose Exponentials
//            dx = delta_center * Math.cos(robotPose.heading) - delta_perp * Math.sin(robotPose.heading);
//            dy = delta_center * Math.sin(robotPose.heading) + delta_perp * Math.cos(robotPose.heading);
//        }
//
//        // Update the robots position
//        robotPose.x -= dx;
//        robotPose.y -= dy;
////        robotPose.heading += delta_theta;
//
////        Pose tagPosition = AprilTagDetection.getGlobalPos(Actuation.getLLResult().getFiducialResults());
////        if (tagPosition != null) {
////            robotPose = new Pose(tagPosition);
////        }
//        org.firstinspires.ftc.teamcode.utility.localization.IMUControl IMUControl;
//        if (IMUControl.isInitialized()) {
//            robotPose.heading = IMUControl.getHeading();
//        }
//
//        prev_ticks_back = ticks_back;
//        prev_ticks_left = ticks_left;
//        prev_ticks_right = ticks_right;
        PinpointControl.updatePose();
        OttoCore.robotPose = PinpointControl.getPose();
    }

    /**
     * Sets motor powers to move in the direction of a point based on PID coefficients defined in ActuationConstants.
     * @param targetPose Robot's target pose
     * @param movementSpeed Robot's move speed
     * @param turnSpeed Robot's turn speed
     */
    public static void moveTowards(Pose targetPose, double movementSpeed, double turnSpeed) {
        // Calculate PID
        double vertPID = ActuationConstants.Movement.verticalPID.calculateSignal(targetPose.x, OttoCore.robotPose.x) * movementSpeed;
        double latPID = ActuationConstants.Movement.lateralPID.calculateSignal(targetPose.y, OttoCore.robotPose.y) * movementSpeed;
        double rotPID = ActuationConstants.Movement.rotationalPID.calculateSignal(targetPose.heading, OttoCore.robotPose.heading) * turnSpeed;

        // Apply feedforward to stop help against friction (if PID signal is less than 0.1, robot might not move, therefore add the smallest power in order to get the robot to move)
        // The 0.1 value and feedforward values might change depending on the robot (specifically weight)
        double vertFF = (Math.abs(targetPose.x-OttoCore.robotPose.x) > 0.2) ? Math.signum(vertPID) * ActuationConstants.Movement.verticalFF : 0.0;
        double latFF = (Math.abs(targetPose.y-OttoCore.robotPose.y) > 0.2) ? Math.signum(latPID) * ActuationConstants.Movement.lateralFF : 0.0;
        double rotFF = (Math.abs(targetPose.heading-OttoCore.robotPose.heading) > 0.2) ? Math.signum(rotPID) * ActuationConstants.Movement.rotationalFF : 0.0;

        double vertSignal = vertPID + vertFF;
        double latSignal = latPID + latFF;
        double rotSignal = rotPID + rotFF;

        Actuation.drive(vertSignal * Math.cos(robotPose.heading) + latSignal * Math.sin(robotPose.heading),
                              rotSignal,
                        vertSignal * Math.sin(robotPose.heading) - latSignal * Math.cos(robotPose.heading));

        Actuation.packet.put("xSignal", vertPID);
        Actuation.packet.put("ySignal", latPID);
        Actuation.packet.put("hSignal", rotPID);
        Actuation.packet.put("X", OttoCore.robotPose.x);
        Actuation.packet.put("Y", OttoCore.robotPose.y);
        Actuation.packet.put("H", OttoCore.robotPose.heading);

        Actuation.updateTelemetry();
    }

    public static double getMove(Pose targetPose, double movementSpeed) {
        // Calculate PID
        double vertPID = ActuationConstants.Movement.verticalPID.calculateSignal(targetPose.x, OttoCore.robotPose.x) * movementSpeed;
        double latPID = ActuationConstants.Movement.lateralPID.calculateSignal(targetPose.y, OttoCore.robotPose.y) * movementSpeed;

        // Apply feedforward to stop help against friction (if PID signal is less than 0.1, robot might not move, therefore add the smallest power in order to get the robot to move)
        // The 0.1 value and feedforward values might change depending on the robot (specifically weight)
        double vertFF = (Math.abs(targetPose.x-OttoCore.robotPose.x) > 0.2) ? Math.signum(vertPID) * ActuationConstants.Movement.verticalFF : 0.0;
        double latFF = (Math.abs(targetPose.y-OttoCore.robotPose.y) > 0.2) ? Math.signum(latPID) * ActuationConstants.Movement.lateralFF : 0.0;

        double vertSignal = vertPID + vertFF;
        double latSignal = latPID + latFF;

        return vertSignal * Math.cos(robotPose.heading) + latSignal * Math.sin(robotPose.heading);
    }

    public static double getStrafe(Pose targetPose, double movementSpeed) {
        // Calculate PID
        double vertPID = ActuationConstants.Movement.verticalPID.calculateSignal(targetPose.x, OttoCore.robotPose.x) * movementSpeed;
        double latPID = ActuationConstants.Movement.lateralPID.calculateSignal(targetPose.y, OttoCore.robotPose.y) * movementSpeed;

        // Apply feedforward to stop help against friction (if PID signal is less than 0.1, robot might not move, therefore add the smallest power in order to get the robot to move)
        // The 0.1 value and feedforward values might change depending on the robot (specifically weight)
        double vertFF = (Math.abs(targetPose.x-OttoCore.robotPose.x) > 0.2) ? Math.signum(vertPID) * ActuationConstants.Movement.verticalFF : 0.0;
        double latFF = (Math.abs(targetPose.y-OttoCore.robotPose.y) > 0.2) ? Math.signum(latPID) * ActuationConstants.Movement.lateralFF : 0.0;

        double vertSignal = vertPID + vertFF;
        double latSignal = latPID + latFF;

        return vertSignal * Math.sin(robotPose.heading) - latSignal * Math.cos(robotPose.heading);
    }

    public static double getTurn(Pose targetPose, double turnSpeed) {
        // Calculate PID
        double rotPID = ActuationConstants.Movement.rotationalPID.calculateSignal(targetPose.heading, OttoCore.robotPose.heading) * turnSpeed;

        // Apply feedforward to stop help against friction (if PID signal is less than 0.1, robot might not move, therefore add the smallest power in order to get the robot to move)
        // The 0.1 value and feedforward values might change depending on the robot (specifically weight)
        double rotFF = (Math.abs(targetPose.heading-OttoCore.robotPose.heading) > 0.2) ? Math.signum(rotPID) * ActuationConstants.Movement.rotationalFF : 0.0;

        return rotPID + rotFF;
    }

    public static Pose getVelocity() {
        PinpointControl.updateVelocityPose();
        return PinpointControl.getVelocityPose();
    }

    public static Pose relativeTransform(Pose reference, double distFor, double distLat, double distRot) {
        double x = reference.x + distFor*Math.cos(reference.heading) + distLat*Math.sin(reference.heading);
        double y = reference.y + distFor*Math.sin(reference.heading) + distLat*Math.cos(reference.heading);
        double h = reference.heading + distRot;

        return new Pose(x, y, h);
    }

    /**
     * Call when target changes for movement to dissolve integral term buildup
     */
    public static void resetMovementPID() {
        ActuationConstants.Movement.verticalPID.reset();
        ActuationConstants.Movement.lateralPID.reset();
        ActuationConstants.Movement.rotationalPID.reset();
    }

    /**
     * Displays the robot's position on the FTC dashboard
     */
    public static void displayPosition(){
        double[] xs = {(SIDE_LENGTH * Math.cos(robotPose.heading) - SIDE_LENGTH * Math.sin(robotPose.heading)) + robotPose.x,
                (-SIDE_LENGTH * Math.cos(robotPose.heading) - SIDE_LENGTH * Math.sin(robotPose.heading)) + robotPose.x,
                (-SIDE_LENGTH * Math.cos(robotPose.heading) + SIDE_LENGTH * Math.sin(robotPose.heading)) + robotPose.x,
                (SIDE_LENGTH * Math.cos(robotPose.heading) + SIDE_LENGTH * Math.sin(robotPose.heading)) + robotPose.x,
                Math.cos(robotPose.heading) * SIDE_LENGTH + robotPose.x};

        // ys refers to multiple ys
        double[] ys = {(SIDE_LENGTH * Math.sin(robotPose.heading) + SIDE_LENGTH * Math.cos(robotPose.heading)) + robotPose.y,
                (-SIDE_LENGTH * Math.sin(robotPose.heading) + SIDE_LENGTH * Math.cos(robotPose.heading)) + robotPose.y,
                (-SIDE_LENGTH * Math.sin(robotPose.heading) - SIDE_LENGTH * Math.cos(robotPose.heading)) + robotPose.y,
                (SIDE_LENGTH * Math.sin(robotPose.heading) - SIDE_LENGTH * Math.cos(robotPose.heading)) + robotPose.y,
                Math.sin(robotPose.heading) * SIDE_LENGTH + robotPose.y};

        Actuation.packet.fieldOverlay().fillPolygon(xs, ys).setFill("blue");

        Actuation.packet.fieldOverlay().setStroke("white");
        Actuation.packet.fieldOverlay().strokeLine(robotPose.x, robotPose.y, xs[4], ys[4]);

        Actuation.updateTelemetry();
    }
}