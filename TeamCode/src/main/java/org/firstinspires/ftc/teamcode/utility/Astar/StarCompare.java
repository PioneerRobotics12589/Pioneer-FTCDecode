package org.firstinspires.ftc.teamcode.utility.Astar;

import java.util.Comparator;

class StarCompare implements Comparator<Star> {
    public int compare(Star s1, Star s2){
        if(s1.fCost() == s2.fCost())
            return Double.compare(s1.hCost, s2.hCost);
        return Double.compare(s1.fCost(), s2.fCost());
    }
}