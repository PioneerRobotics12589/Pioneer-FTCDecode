package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.List;

public class OttoCore {
    static final int SIDE_LENGTH = 6;

    public static Pose robotPose, lastPose;
    public static double ticks_left, ticks_right, ticks_back;

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

        // Initialize PID controllers with proper gains
        vertical = new PIDController(ActuationConstants.Movement.verticalGains);
        lateral = new PIDController(ActuationConstants.Movement.lateralGains);
        rotational = new PIDController(ActuationConstants.Movement.rotationalGains);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        lastTime = System.nanoTime();
        lastPose = new Pose(robotPose);
    }

    /**
     * Updates the robot's pose based off of encoder values from odometry
     */
    public static void updatePosition() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        ticks_left = -Actuation.frontLeft.getCurrentPosition();
        ticks_right = Actuation.frontRight.getCurrentPosition();
        ticks_back = Actuation.backRight.getCurrentPosition();

        double delta_ticks_left = (ticks_left - prev_ticks_left);
        double delta_ticks_right = (ticks_right - prev_ticks_right);
        double delta_ticks_back = (ticks_back - prev_ticks_back);

        dtheta = ((delta_ticks_left - delta_ticks_right) / ActuationConstants.Drivetrain.track_width) * ActuationConstants.Drivetrain.scale;
        dx_center = ((delta_ticks_left + delta_ticks_right) / 2) * ActuationConstants.Drivetrain.scale * ActuationConstants.Drivetrain.centerMultiplier;
        dx_perpendicular = -1 * (delta_ticks_back - (ActuationConstants.Drivetrain.forward_offset * ((delta_ticks_left - delta_ticks_right) / ActuationConstants.Drivetrain.track_width))) * ActuationConstants.Drivetrain.scale * ActuationConstants.Drivetrain.perpendicularMultiplier;

        if(dtheta != 0) {
            dx = (dx_center * (Math.sin(dtheta) * Math.cos(robotPose.heading) - Math.sin(robotPose.heading) * (-Math.cos(dtheta) + 1)) + dx_perpendicular * (Math.cos(robotPose.heading) * (Math.cos(dtheta) - 1) - Math.sin(dtheta) * Math.sin(robotPose.heading))) / dtheta;
            dy = (dx_center * (Math.sin(dtheta) * Math.sin(robotPose.heading) + Math.cos(robotPose.heading) * (-Math.cos(dtheta) + 1)) + dx_perpendicular * (Math.sin(robotPose.heading) * (Math.cos(dtheta) - 1) + Math.sin(dtheta) * Math.cos(robotPose.heading))) / dtheta;
        }
        else {
            dx = dx_center * Math.cos(robotPose.heading) - dx_perpendicular * Math.sin(robotPose.heading);
            dy = dx_center * Math.sin(robotPose.heading) + dx_perpendicular * Math.cos(robotPose.heading);
        }
        robotPose.x += dx;
        robotPose.y += dy;
        robotPose.heading += -1 * dtheta;

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

        double clampVert = Math.max(-1, Math.min(1, vertSignal));
        double clampLat = Math.max(-1, Math.min(1, latSignal));
        double clampRot = Math.max(-1, Math.min(1, rotSignal));

        double move = clampVert * Math.cos(robotPose.heading) + clampLat * Math.sin(robotPose.heading);
        double strafe = clampVert * Math.sin(robotPose.heading) - clampLat * Math.cos(robotPose.heading);

//        Actuation.packet.put("vertical signal", vertSignal);
//        Actuation.packet.put("lateral signal", latSignal);
//        Actuation.packet.put("rotational signal", rotSignal);
//        Actuation.updateTelemetry();

        double voltageComp = 12 / voltageSensor.getVoltage();

        Actuation.drive(move * movementSpeed * voltageComp, clampRot * turnSpeed * voltageComp, strafe * movementSpeed * voltageComp);
    }

    public static double[] moveTowardsSignals(Pose targetPose, double movementSpeed, double turnSpeed) {
        // Update coefficients in case changed in dashboard
        lateral.updateCoeffs(ActuationConstants.Movement.lateralGains);
        vertical.updateCoeffs(ActuationConstants.Movement.verticalGains);
        rotational.updateCoeffs(ActuationConstants.Movement.rotationalGains);

        double vertSignal = vertical.calculateSignal(targetPose.x, OttoCore.robotPose.x);
        double latSignal = lateral.calculateSignal(targetPose.y, OttoCore.robotPose.y);
        double rotSignal = rotational.calculateSignal(targetPose.heading, OttoCore.robotPose.heading);

        double clampVert = Math.max(-1, Math.min(1, vertSignal));
        double clampLat = Math.max(-1, Math.min(1, latSignal));
        double clampRot = Math.max(-1, Math.min(1, rotSignal));

        double move = clampVert * Math.cos(robotPose.heading) + clampLat * Math.sin(robotPose.heading);
        double strafe = clampVert * Math.sin(robotPose.heading) - clampLat * Math.cos(robotPose.heading);

        double voltageComp = 12 / voltageSensor.getVoltage();

        return new double[] {move * movementSpeed * voltageComp, clampRot * turnSpeed * voltageComp, strafe * movementSpeed * voltageComp};
    }

    public static Pose getVelocity() {
        double dt = (System.nanoTime() - lastTime)/1000000000.00;

        Pose vel = new Pose((robotPose.x - lastPose.x)/dt, (robotPose.y - lastPose.y)/dt, (robotPose.heading - lastPose.heading)/dt);

        lastPose = new Pose(robotPose);
        lastTime = System.nanoTime();
        return vel;
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