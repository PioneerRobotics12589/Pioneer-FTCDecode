package org.firstinspires.ftc.teamcode.utility.dataTypes;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.localization.PinpointControl;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class Trajectory {
    double moveSpeed, turnSpeed;
    Pose start;

    ArrayList<Runnable> movements;
    ArrayList<Runnable> periodicTasks;

    /**
     * Initializes a new trajectory
     */
    public Trajectory() {
        movements = new ArrayList<>();
        periodicTasks = new ArrayList<>();

        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;
        start = OttoCore.robotPose;
    }

    /**
     * Initializes a new trajectory with unique starting pose
     * @param startPose Robot's starting position & heading
     */
    public Trajectory(Pose startPose) {
        movements = new ArrayList<>();
        periodicTasks = new ArrayList<>();

        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;

        start = startPose;
    }

    /**
     * Move the robot to another pose
     * @param targetPose The robot's targeted pose
     * @return Parent trajectory
     */
    public Trajectory lineTo(Pose targetPose) {
        movements.add(() -> runLineTo(targetPose));
        return this;
    }
    public Trajectory lineThrough(Pose targetPose) {
        movements.add(() -> runLineThrough(targetPose));
        return this;
    }
    public Trajectory lineToPrecise(Pose targetPose) {
        movements.add(() -> runLineToPrecise(targetPose));
        return this;
    }
    public Trajectory lineToTeleOp(Pose targetPose, BooleanSupplier gamepadButton) {
        movements.add(() -> runLineToTeleOp(targetPose, gamepadButton));
        return this;
    }
    public Trajectory addPeriodic(Runnable task) {
        periodicTasks.add(task);
        return this;
    }
    public Trajectory stopPeriodic(Runnable task) {
        periodicTasks.remove(task);
        return this;
    }
    private void runPeriodics() {
        for (Runnable task : periodicTasks) {
            task.run();
        }
    }

    /**
     * Move the robot to another pose with a specific speed
     * @param targetPose The robot's targeted pose
     * @param moveSpeed The robot's target move speed
     * @param turnSpeed The robot's target turn speed
     * @return Parent trajectory
     */
    public Trajectory lineTo(Pose targetPose, double moveSpeed, double turnSpeed) {
        movements.add(() -> runLineTo(targetPose, moveSpeed, turnSpeed));
        return this;
    }
    public Trajectory lineThrough(Pose targetPose, double moveSpeed, double turnSpeed) {
        movements.add(() -> runLineThrough(targetPose, moveSpeed, turnSpeed));
        return this;
    }
    public Trajectory lineToPrecise(Pose targetPose, double moveSpeed, double turnSpeed) {
        movements.add(() -> runLineToPrecise(targetPose, moveSpeed, turnSpeed));
        return this;
    }
    public Trajectory lineToTeleOp(Pose targetPose, double moveSpeed, double turnSpeed, BooleanSupplier gamepadButton) {
        movements.add(() -> runLineToTeleOp(targetPose, moveSpeed, turnSpeed, gamepadButton));
        return this;
    }

    /**
     * Specifies an action for the robot to carry out
     * @param action Runnable robot action
     * @return Parent trajectory
     */
    public Trajectory action(Runnable action) {
        movements.add(action);
        return this;
    }

    /**
     * Builds and runs the trajectory's previously specified movements
     */
    public void run() {
        for (Runnable movement : movements) {
            movement.run();
        }
    }

    private void runLineTo(Pose targetPose) {
        runLineTo(targetPose, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
    }
    private void runLineThrough(Pose targetPose) {
        runLineThrough(targetPose, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
    }
    private void runLineToPrecise(Pose targetPose) {
        runLineToPrecise(targetPose, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed);
    }
    private void runLineToTeleOp(Pose targetPose, BooleanSupplier gamepadButton) {
        runLineToTeleOp(targetPose, ActuationConstants.Autonomous.moveSpeed, ActuationConstants.Autonomous.turnSpeed, gamepadButton);
    }
    private void runLineTo(Pose targetPose, double mSpeed, double tSpeed) {
        OttoCore.resetMovementPID();
        while (targetPose.heading - OttoCore.robotPose.heading > Math.PI) OttoCore.robotPose.heading += 2 * Math.PI;
        while (targetPose.heading - OttoCore.robotPose.heading < -Math.PI) OttoCore.robotPose.heading -= 2 * Math.PI;

        double vel = 1;
        double rotVel = 1;

        boolean hasRun = false;

        Pose center = new Pose(0, 0, 0);
        boolean withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
        boolean withinRange = OttoCore.robotPose.withinRange(targetPose, 0.5, 0.5, Math.toRadians(2));

        while(!(vel == 0 && Math.abs(rotVel) < 0.5 && hasRun && withinRange && withinField)) {
            OttoCore.updatePosition();
            runPeriodics();
            OttoCore.displayPosition();

            Actuation.packet.put("Robot Pos", OttoCore.robotPose);
            Actuation.packet.put("vel", vel);
            Actuation.packet.put("rot vel", rotVel);
            Actuation.updateTelemetry();

            OttoCore.moveTowards(targetPose, mSpeed, tSpeed);

            Pose robot_vel = OttoCore.getVelocity();
            vel = Math.sqrt(Math.pow(robot_vel.x, 2) + Math.pow(robot_vel.y, 2));
            rotVel = robot_vel.heading;
            if (vel != 0 || rotVel != 0) hasRun = true;

            withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
            withinRange = OttoCore.robotPose.withinRange(targetPose, 0.5, 0.5, Math.toRadians(2));
        }

        Actuation.drive(0.0, 0.0, 0.0);
    }
    private void runLineThrough(Pose targetPose, double mSpeed, double tSpeed) {
        OttoCore.resetMovementPID();
        while (targetPose.heading - OttoCore.robotPose.heading > Math.PI) OttoCore.robotPose.heading += 2 * Math.PI;
        while (targetPose.heading - OttoCore.robotPose.heading < -Math.PI) OttoCore.robotPose.heading -= 2 * Math.PI;

        double vel;
        double rotVel;

        boolean hasRun = false;

        Pose center = new Pose(0, 0, 0);
        boolean withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
        boolean withinRange = OttoCore.robotPose.withinRange(targetPose, 2, 2, Math.toRadians(5));

        while(!(hasRun && withinRange && withinField)) {
            OttoCore.updatePosition();
            runPeriodics();
            OttoCore.displayPosition();

            Actuation.packet.put("Robot Pos", OttoCore.robotPose);
            Actuation.updateTelemetry();

            Pose robot_vel = OttoCore.getVelocity();
            vel = Math.sqrt(Math.pow(robot_vel.x, 2) + Math.pow(robot_vel.y, 2));
            rotVel = robot_vel.heading;
            if (vel != 0 || rotVel != 0) hasRun = true;

            double angle = Math.atan2(targetPose.y - OttoCore.robotPose.y, targetPose.x - OttoCore.robotPose.x);
            double rotSignal = OttoCore.getTurn(targetPose, tSpeed);

            // Normalize based on the larger distance (strafe multiplied by 1.2 to match move speed)
            double moveSignal = mSpeed * Math.cos(angle - OttoCore.robotPose.heading);
            double strafeSignal = mSpeed * Math.sin(angle - OttoCore.robotPose.heading) * 1.2;

            double clampRot = Math.max(-1.0, Math.min(1, rotSignal));
            double clampMove = Math.max(-1.0, Math.min(1, moveSignal));
            double clampStrafe = Math.max(-1.0, Math.min(1, strafeSignal));

            Actuation.drive(clampMove, clampRot, clampStrafe);

            withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
            withinRange = OttoCore.robotPose.withinRange(targetPose, 2, 2, Math.toRadians(5));
        }
    }
    private void runLineToPrecise(Pose targetPose, double mSpeed, double tSpeed) {
        OttoCore.resetMovementPID();
        while (targetPose.heading - OttoCore.robotPose.heading > Math.PI) OttoCore.robotPose.heading += 2 * Math.PI;
        while (targetPose.heading - OttoCore.robotPose.heading < -Math.PI) OttoCore.robotPose.heading -= 2 * Math.PI;

        double vel = 1;
        double rotVel = 1;

        boolean hasRun = false;

        Pose center = new Pose(0, 0, 0);
        boolean withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
        boolean withinRange = OttoCore.robotPose.withinRange(targetPose, 0.25, 0.25, Math.toRadians(1));

        while(!(vel == 0 && Math.abs(rotVel) < 0.5 && hasRun && withinRange && withinField)) {
            OttoCore.updatePosition();
            runPeriodics();
            OttoCore.displayPosition();

            Actuation.packet.put("Robot Pos", OttoCore.robotPose);
            Actuation.packet.put("vel", vel);
            Actuation.packet.put("rot vel", rotVel);
            Actuation.updateTelemetry();

            OttoCore.moveTowards(targetPose, mSpeed, tSpeed);

            Pose robot_vel = OttoCore.getVelocity();
            vel = Math.sqrt(Math.pow(robot_vel.x, 2) + Math.pow(robot_vel.y, 2));
            rotVel = robot_vel.heading;
            if (vel != 0 || rotVel != 0) hasRun = true;

            withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
            withinRange = OttoCore.robotPose.withinRange(targetPose, 0.25, 0.25, Math.toRadians(1));
        }

        Actuation.drive(0.0, 0.0, 0.0);
    }

    /**
     * runLineTo, but the TeleOp driver can stop the path
     * @param targetPose target pose
     * @param mSpeed move speed
     * @param tSpeed turn speed
     * @param gamepadButton wasPressed() function for the gamepad button to stop the path
     */
    private void runLineToTeleOp(Pose targetPose, double mSpeed, double tSpeed, BooleanSupplier gamepadButton) {
        OttoCore.resetMovementPID();
        while (targetPose.heading - OttoCore.robotPose.heading > Math.PI) OttoCore.robotPose.heading += 2 * Math.PI;
        while (targetPose.heading - OttoCore.robotPose.heading < -Math.PI) OttoCore.robotPose.heading -= 2 * Math.PI;

        double vel = 1;
        double rotVel = 1;

        boolean hasRun = false;

        Pose center = new Pose(0, 0, 0);
        boolean withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
        boolean withinRange = OttoCore.robotPose.withinRange(targetPose, 1, 1, Math.toRadians(2));
        boolean stopPath = gamepadButton.getAsBoolean();

        while(!(vel == 0 && rotVel == 0 && hasRun && withinRange && withinField && !stopPath)) {
            OttoCore.updatePosition();
            runPeriodics();
            OttoCore.displayPosition();

            Actuation.packet.put("Robot Pos", OttoCore.robotPose);
            Actuation.packet.put("vel", vel);
            Actuation.packet.put("rot vel", rotVel);
            Actuation.updateTelemetry();

            OttoCore.moveTowards(targetPose, mSpeed, tSpeed);

            Pose robot_vel = OttoCore.getVelocity();
            vel = Math.sqrt(Math.pow(robot_vel.x, 2) + Math.pow(robot_vel.y, 2));
            rotVel = robot_vel.heading;
            if (vel != 0 || rotVel != 0) hasRun = true;

            withinField = OttoCore.robotPose.withinRange(center, 72, 72, Math.toRadians(360));
            withinRange = OttoCore.robotPose.withinRange(targetPose, 1, 1, Math.toRadians(2));
            stopPath = gamepadButton.getAsBoolean();
        }

        Actuation.drive(0.0, 0.0, 0.0);
    }
}