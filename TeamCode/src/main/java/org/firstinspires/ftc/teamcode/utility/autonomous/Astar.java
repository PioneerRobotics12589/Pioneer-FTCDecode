package org.firstinspires.ftc.teamcode.utility.autonomous;

import org.firstinspires.ftc.teamcode.utility.dataTypes.*;
import java.util.*;

public class Astar{

    // creates a grid of Nodes which represent global positions
    static final int gridWidth = 144;
    static final int gridLength = 144;
    private static final Star[][] graph = new Star[gridLength][gridWidth];
    public static CollisionBox[] obstacles = new CollisionBox[1];
    static {obstacles[0] = new CollisionBox(20,0,2,2);}
    /* add obstacles here with the CollisionBox class, the algorithm automatically avoids
     these placed obstacles */

    private static List<Star> getNeighbors(Star center){
        List<Star> neighbors = new ArrayList<>();
        for(int y = -1; y <= 1; ++y){
            for(int x = -1; x <= 1; ++x){

                if(x == 0  && y == 0) continue;
                if(center.x+ x >= 0 && center.x+ x < gridWidth && center.y + y >= 0 && center.y + y < gridLength){
                    neighbors.add(graph[center.y+ y][center.x + x]);
                }
            }
        }
        return neighbors;
    }
    //getNeighbors() gets the neighboring Star objects of a Star object in the grid
    public static ArrayList<Pose> pathFind(int startX, int startY, int endX, int endY){
        Star start = new Star(startX, startY);
        Star goal = new Star(endX,endY);

        int x, y;
        for(y=0; y<graph.length; ++y)
            for(x=0; x<graph[0].length; ++x){
                graph[y][x] = new Star(x, y);
            }

        graph[startY][startX] = start;
        graph[endY][endX] = goal;

        PriorityQueue<Star> open = new PriorityQueue<>(8, new StarCompare());
        HashSet<Star> closed = new HashSet<>();
        open.add(start);

        Star current = null;
        //put robot dimensions here.
        CollisionBox currentBox = new CollisionBox(startX, startY, 14.25, 17.25);

        while(!open.isEmpty()){
            current = open.remove();
            if(current == goal) break;
            currentBox.updateBox(current.x, current.y);
            closed.add(current);

            for(Star neighbor: getNeighbors(current)){
                double edge = current.distanceTo(neighbor);

                currentBox.updateBox(neighbor.x, neighbor.y);
                boolean collision = false;
                for(CollisionBox box : obstacles){
                    if(currentBox.collidesWith(box)){
                        collision = true;
                        break;
                    }

                }
                if(collision) continue;

                if(!open.contains(neighbor) && !closed.contains(neighbor)){
                    neighbor.gCost = current.gCost + edge;
                    neighbor.hCost = neighbor.distanceTo(goal);
                    neighbor.parent = current;
                    open.add(neighbor);
                }else if(!closed.contains(neighbor) && neighbor.gCost > current.gCost + edge){
                    neighbor.gCost = current.gCost + edge;
                    neighbor.parent = current;
                }
            }
        }

        ArrayList<Pose> path = new ArrayList<>();
        double prevSlope = Double.NEGATIVE_INFINITY;
        while(current != start){
            double slope = current.slopeWith(current.parent);
            if(slope != prevSlope){
                path.add(current.parent.toPose());
                prevSlope = slope;
            }
            current = current.parent;
        }

        Collections.reverse(path);
        return path;
    }
}