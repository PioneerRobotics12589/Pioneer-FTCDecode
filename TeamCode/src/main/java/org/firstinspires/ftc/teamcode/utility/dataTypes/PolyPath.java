package org.firstinspires.ftc.teamcode.utility.dataTypes;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;

public class PolyPath {
    private final double[][] coefficients;
    private final ElapsedTime runtime = new ElapsedTime();
    private final Pose endPose;
    private final double endTime;

    public PolyPath(double[][] coeffs, Pose endPose, double endTime) {
        coefficients = coeffs;
        this.endPose = endPose;
        this.endTime = endTime;
    }

    /**
     * Calculates movement values from the prespecified polynomial functions
     * @param time Current time
     * @return
     */
    private double[] motionValues(double time) {
        double[] values = new double[3];

        // X - Velocity
        for (int x_coeff = 0; x_coeff < coefficients[0].length; x_coeff++) {
            values[0] += coefficients[0][x_coeff]*Math.pow(time, x_coeff);
        }

        // Y - Velocity
        for (int y_coeff = 0; y_coeff < coefficients[1].length; y_coeff++) {
            values[1] += coefficients[1][y_coeff]*Math.pow(time, y_coeff);
        }

        // Heading
        for (int head_coeff = 0; head_coeff < coefficients[2].length; head_coeff++) {
            values[2] += coefficients[2][head_coeff]*Math.pow(time, head_coeff);
        }

        return values;
    }
    private static double clamp(double value) {
        return Math.max(-1, Math.min(1, value));
    }
    private double[] toMotionPoint(double[] motionPoint) {
        double[] powers = new double[3];

        double[] velocity = getVelocity();
        OttoCore.updatePosition();
        double heading = OttoCore.robotPose.heading;

        if (motionPoint[2] > heading) {
            while (Math.abs(motionPoint[2] - heading) > Math.toRadians(180)) {
                heading += 2 * Math.PI;
            }
        } else if (motionPoint[2] < heading) {
            while (Math.abs(motionPoint[2] - heading) > Math.toRadians(180)) {
                heading -= 2 * Math.PI;
            }
        }

        double vX_signal = ActuationConstants.ModelPID.vX_PID.calculateSignal(motionPoint[0], velocity[0]);
        double vY_signal = ActuationConstants.ModelPID.vY_PID.calculateSignal(motionPoint[1], velocity[1]);
        double head_signal = ActuationConstants.ModelPID.head_PID.calculateSignal(motionPoint[2], heading);

        powers[0] = clamp(vX_signal) * Math.cos(heading) + clamp(vY_signal) * Math.sin(heading);
        powers[1] = clamp(vX_signal) * Math.sin(heading) - clamp(vY_signal) * Math.cos(heading);
        powers[2] = clamp(head_signal);
        return powers;
    }
    private void toPose(Pose pose) {
        if (pose.heading > OttoCore.robotPose.heading) {
            while (Math.abs(pose.heading - OttoCore.robotPose.heading) > Math.toRadians(180)) {
                OttoCore.robotPose.heading += 2 * Math.PI;
            }
        } else if (pose.heading < OttoCore.robotPose.heading) {
            while (Math.abs(pose.heading - OttoCore.robotPose.heading) > Math.toRadians(180)) {
                OttoCore.robotPose.heading -= 2 * Math.PI;
            }
        }

        while(!OttoCore.robotPose.withinRange(pose, 0.25, 0.25, Math.toRadians(3))) {
            OttoCore.updatePosition();
            OttoCore.moveTowards(pose, 1.0, 1.0);
        }

        Actuation.drive(0, 0, 0);
    }
    private double[] getVelocity() {
        double[] velocities = new double[2];

        OttoCore.updatePosition();

        Pose p1 = new Pose(OttoCore.robotPose);
        double time = runtime.seconds();

        // Internal delay

        Pose p2 = new Pose(OttoCore.robotPose);
        double dt = runtime.seconds()-time;

        velocities[0] = (p2.x-p1.x)/dt;
        velocities[1] = (p2.y-p1.y)/dt;

        return velocities;
    }
    public void runPath() {
        runtime.reset();
        double time = runtime.seconds();

        while (time < endTime) {
            double[] values = toMotionPoint(motionValues(time)); // Find motor values dependent on the time
            Actuation.drive(values[0], values[2], values[1]);
            time = runtime.seconds();
        }

        toPose(endPose); // Adjust for possible error after path
    }
}