package org.firstinspires.ftc.teamcode.utility.autonomous;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

public class Trajectory {
    double moveSpeed, turnSpeed;

    ArrayList<Runnable> movements;

    /**
     * Initializes a new trajectory
     */
    public Trajectory() {
        movements = new ArrayList<>();

        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;
    }

    /**
     * Initializes a new trajectory with unique starting pose
     * @param startPose Robot's starting position & heading
     */
    public Trajectory(Pose startPose) {
        movements = new ArrayList<>();

        moveSpeed = ActuationConstants.Autonomous.moveSpeed;
        turnSpeed = ActuationConstants.Autonomous.turnSpeed;

        OttoCore.robotPose = startPose;
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

    private void runLineTo(Pose targetPose, double mSpeed, double tSpeed) {
        while(!OttoCore.robotPose.withinRange(targetPose, 0.5, 0.5, Math.toRadians(5))) {
            OttoCore.updatePosition();
            OttoCore.displayPosition();

            OttoCore.moveTowards(targetPose, mSpeed, tSpeed);
        }

        Actuation.drive(0, 0, 0);
    }
}
