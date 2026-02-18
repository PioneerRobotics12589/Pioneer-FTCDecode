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
import org.firstinspires.ftc.teamcode.utility.imu.IMUControl;

import java.util.List;
public class OttoCore {
    static final int SIDE_LENGTH = 6;

    public static Pose robotPose, lastPose;
    public static double ticks_left = 0, ticks_right = 0, ticks_back = 0;

    static double dx, dy, dtheta;
    static double dx_center, dx_perpendicular;
    static double prev_ticks_left, prev_ticks_right, prev_ticks_back;

    static PIDController vertical, lateral, rotational;

    static List<LynxModule> allHubs;
    static VoltageSensor voltageSensor;

    static long lastTime;

    /**
     * Necessary in order to correctly initialize reading from odometry and accessing voltage sensor
     * @param hardwareMap Current hardware map
     */
    public static void setup(HardwareMap hardwareMap) {
        robotPose = new Pose(0, 0, 0);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        IMUControl.setup(hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        IMUControl.setYaw(0);

        // Initialize PID controllers with proper gains
        vertical = new PIDController(ActuationConstants.Movement.verticalGains);
        lateral = new PIDController(ActuationConstants.Movement.lateralGains);
        rotational = new PIDController(ActuationConstants.Movement.rotationalGains);

        ticks_right = 0; ticks_left = 0; ticks_back = 0;
        prev_ticks_right = 0; prev_ticks_left = 0; prev_ticks_back = 0;
        dx = 0; dy = 0; dtheta = 0; dx_center = 0; dx_perpendicular = 0;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        lastTime = System.nanoTime();
        lastPose = new Pose(robotPose);
    }

    /**
     * Updates the robot's pose based off of encoder values from odometry
     */
    // All equations derived from and explained by:
    // https://gm0.org/en/latest/docs/software/concepts/odometry.html
    public static void updatePosition() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        lastPose = new Pose(OttoCore.robotPose);
        lastTime = System.nanoTime();

        ticks_left = -Actuation.frontLeft.getCurrentPosition();
        ticks_right = Actuation.frontRight.getCurrentPosition();
        ticks_back = Actuation.backRight.getCurrentPosition();

        double delta_ticks_left = (ticks_left - prev_ticks_left);
        double delta_ticks_right = (ticks_right - prev_ticks_right);
        double delta_ticks_back = (ticks_back - prev_ticks_back);

        double inchesPerTick = ActuationConstants.Drivetrain.wheel_circ / ActuationConstants.Drivetrain.ticksPerRev;

        // We divide each differential by ticks per revolution and multiply by the wheel circumference in order to account for real-world distance
        delta_ticks_left *= inchesPerTick;
        delta_ticks_right *= inchesPerTick;
        delta_ticks_back *= inchesPerTick;

        // Change in angle
        double delta_theta = ((delta_ticks_left - delta_ticks_right) / ActuationConstants.Drivetrain.track_width);

        // Change in the center position of the robot relative to itself (just the average of the parallel wheel diffs)
        // aka Vertical displacement
        double delta_center = ((delta_ticks_left + delta_ticks_right) / 2);

        // Change in the perpendicular position of the robot relative to itself
        // aka Horizontal displacement
        double delta_perp = delta_ticks_back - (ActuationConstants.Drivetrain.forward_offset * delta_theta); // The other formula is negative of this

        double dx = 0;
        double dy = 0;
        if (delta_theta != 0) { // Accounting for division by zero
            dx = (delta_center * (Math.cos(robotPose.heading) * Math.sin(delta_theta) - Math.sin(robotPose.heading) * (1 - Math.cos(delta_theta)))
                    + delta_perp * (Math.cos(robotPose.heading) * (Math.cos(delta_theta) - 1) - Math.sin(robotPose.heading) * Math.sin(delta_theta))) / delta_theta;

            dy = (delta_center * (Math.sin(robotPose.heading) * Math.sin(delta_theta) + Math.cos(robotPose.heading) * (1 - Math.cos(delta_theta)))
                    + delta_perp * (Math.sin(robotPose.heading) * (Math.cos(delta_theta) - 1) + Math.cos(robotPose.heading) * Math.sin(delta_theta))) / delta_theta;
        }
        else { // If delta_theta is 0 we use Euler Integration instead of Pose Exponentials
            dx = delta_center * Math.cos(robotPose.heading) - delta_perp * Math.sin(robotPose.heading);
            dy = delta_center * Math.sin(robotPose.heading) + delta_perp * Math.cos(robotPose.heading);
        }

        // Update the robots position
        robotPose.x -= dx;
        robotPose.y -= dy;
//        robotPose.heading += delta_theta;

//        Pose tagPosition = AprilTagDetection.getGlobalPos(Actuation.getLLResult().getFiducialResults());
//        if (tagPosition != null) {
//            robotPose = new Pose(tagPosition);
//        }
        if (IMUControl.isInitialized()) {
            robotPose.heading = IMUControl.getHeading();
        }

        prev_ticks_back = ticks_back;
        prev_ticks_left = ticks_left;
        prev_ticks_right = ticks_right;
    }

    /**
     * Sets motor powers to move in the direction of a point based on PID coefficients defined in ActuationConstants.
     * @param targetPose Robot's target pose
     * @param movementSpeed Robot's move speed
     * @param turnSpeed Robot's turn speed
     */
    public static void moveTowards(Pose targetPose, double movementSpeed, double turnSpeed) {
        // Update coefficients in case changed in dashboard
        lateral.updateCoeffs(ActuationConstants.Movement.lateralGains);
        vertical.updateCoeffs(ActuationConstants.Movement.verticalGains);
        rotational.updateCoeffs(ActuationConstants.Movement.rotationalGains);

        double vertSignal = vertical.calculateSignal(targetPose.x, OttoCore.robotPose.x);
        double latSignal = lateral.calculateSignal(targetPose.y, OttoCore.robotPose.y);
        double rotSignal = rotational.calculateSignal(targetPose.heading, OttoCore.robotPose.heading);

        double clampVert = -Math.max(-1.0, Math.min(1, vertSignal));
        double clampLat = -Math.max(-1.0, Math.min(1, latSignal));
        double clampRot = Math.max(-1.0, Math.min(1, rotSignal));

        double move = clampVert * Math.cos(robotPose.heading) + clampLat * Math.sin(robotPose.heading);
        double strafe = clampVert * Math.sin(robotPose.heading) - clampLat * Math.cos(robotPose.heading);

//        double voltageComp = 12 / voltageSensor.getVoltage();

        Actuation.drive(move * movementSpeed, clampRot * turnSpeed, strafe * movementSpeed);

        Actuation.packet.put("xSignal", vertSignal);
        Actuation.packet.put("ySignal", latSignal);
        Actuation.packet.put("hSignal", rotSignal);
        Actuation.packet.put("X", OttoCore.robotPose.x);
        Actuation.packet.put("Y", OttoCore.robotPose.y);
        Actuation.packet.put("H", OttoCore.robotPose.heading);

        Actuation.updateTelemetry();
    }

    public static double getMove(Pose targetPose, double movementSpeed) {
        lateral.updateCoeffs(ActuationConstants.Movement.lateralGains);
        vertical.updateCoeffs(ActuationConstants.Movement.verticalGains);

        double vertSignal = vertical.calculateSignal(targetPose.x, OttoCore.robotPose.x);
        double latSignal = lateral.calculateSignal(targetPose.y, OttoCore.robotPose.y);

        double move = vertSignal * Math.sin(robotPose.heading) - latSignal * Math.cos(robotPose.heading);

        return -Math.max(-1.0, Math.min(1, move*movementSpeed));
    }

    public static double getStrafe(Pose targetPose, double movementSpeed) {
        lateral.updateCoeffs(ActuationConstants.Movement.lateralGains);
        vertical.updateCoeffs(ActuationConstants.Movement.verticalGains);

        double vertSignal = vertical.calculateSignal(targetPose.x, OttoCore.robotPose.x);
        double latSignal = lateral.calculateSignal(targetPose.y, OttoCore.robotPose.y);

        double strafe = vertSignal * Math.sin(robotPose.heading) - latSignal * Math.cos(robotPose.heading);

        return -Math.max(-1.0, Math.min(1, strafe*movementSpeed));
    }

    public static double getTurn(Pose targetPose, double turnSpeed) {
        rotational.updateCoeffs(ActuationConstants.Movement.rotationalGains);

        double rotSignal = rotational.calculateSignal(targetPose.heading, OttoCore.robotPose.heading);

        return Math.max(-1.0, Math.min(1, rotSignal*turnSpeed));
    }

    public static Pose getVelocity() {
        double dt = (System.nanoTime() - lastTime)/1000000000.00;

        return new Pose((robotPose.x - lastPose.x)/dt, (robotPose.y - lastPose.y)/dt, (robotPose.heading - lastPose.heading)/dt);
    }

    public static Pose relativeTransform(Pose reference, double distFor, double distLat, double distRot) {
        double x = reference.x + distFor*Math.cos(reference.heading) + distLat*Math.sin(reference.heading);
        double y = reference.y + distFor*Math.sin(reference.heading) + distLat*Math.cos(reference.heading);
        double h = reference.heading + distRot;

        return new Pose(x, y, h);
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