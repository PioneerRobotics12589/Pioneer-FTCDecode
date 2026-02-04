package org.firstinspires.ftc.teamcode.utility.Astar;

public class CollisionBox {
    public int x,y;
    public double length, width;

    //Creates a rectangular collision box
    public CollisionBox(int x, int y, double length, double width){
        this.x = x;
        this.y = y;
        this.length = length;
        this.width = width;
    }

    public void updateBox(int x, int y){
        this.x = x;
        this.y = y;
    }

    //collision only accounts for non rotated rectangles
    public boolean collidesWith(CollisionBox B){
        return (x - length/2 < B.x + B.length/2) && (x + length/2 > B.x - B.length/2) && (y - width/2 < B.y + B.width/2) && (y + width/2 > B.y - B.width/2);
    }
}