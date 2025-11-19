package org.firstinspires.ftc.teamcode.utility.dataTypes;

import androidx.annotation.NonNull;

public class Pose {
    public double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(Point point, double heading) {
        this.x = point.x;
        this.y = point.y;
        this.heading = heading;
    }

    public Pose(Pose newPose) {
        this.x = newPose.x;
        this.y = newPose.y;
        this.heading = newPose.heading;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    /**
     * Checks if another pose is within [-range, range] inches of the parent point
     * @param p Comparison pose
     * @param xRange Range of comparison on the x axis (in)
     * @param yRange Range of comparison on the y axis (in)
     * @param hRange Range of comparison for heading (rad)
     * @return if p is within [-range, range] of pose
     */
    public boolean withinRange(Pose p, double xRange, double yRange, double hRange) {
        boolean withinX = (p.x >= x - xRange && p.x <= x + xRange);
        boolean withinY = (p.y >= y - yRange && p.y <= y + yRange);
        boolean withinHeading = (p.heading >= heading - hRange && p.heading <= heading + hRange);
        return withinX && withinY && withinHeading;
    }

    public Pose augment(Pose newPose) {
        return new Pose(x + newPose.x, y + newPose.y, heading + newPose.heading);
    }

    public Pose augment(double x_, double y_, double h_) {
        return new Pose(x + x_, y + y_, heading + h_);
    }

    @NonNull
    @Override
    public String toString() {
        return "X: " + (int)(x * 100)/100.0 + ", Y: " + (int)(y*100)/100.0 + ", H: " + Math.toDegrees(heading);
    }

    public boolean equals(Pose other) {
        return (x == other.x) && (y == other.y) && (heading == other.heading);
    }
}