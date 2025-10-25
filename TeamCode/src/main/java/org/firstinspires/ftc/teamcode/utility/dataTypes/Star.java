package org.firstinspires.ftc.teamcode.utility.dataTypes;

import androidx.annotation.NonNull;
public class Star {
    // This class creates nodes for the A* algorithm
    public int x,y;
    public Star parent;
    public double gCost, hCost;
    public Star(int x, int y){
        this.x = x;
        this.y = y;
    }
    public Star(Point p){
        x = (int) p.x;
        y = (int) p.y;
    }
    public Star(Pose ps){
        x = (int) ps.x;
        y = (int) ps.y;
    }
    public double fCost(){
        return gCost+hCost;
    }
    public double distanceTo(Star destination){
        int difX = destination.x - x;
        int difY = destination.y - y;
        return Math.sqrt(Math.pow(difX,2) + Math.pow(difY,2));
    }
    public double slopeWith(Star destination){
        if(x == destination.x)
            return Double.POSITIVE_INFINITY;
        return (destination.y - y*1.0) / (destination.x - x*1.0);
    }

    public Point toPoint(){
        return new Point(x,y);
    }
    public Pose toPose(){
        return new Pose(this.x,this.y,Math.toRadians(0));
    }

    @NonNull
    @Override
    public String toString(){
        return "X:" + x + " Y:" + y + " gCost:" + gCost + " hCost:" + hCost;
    }
}
