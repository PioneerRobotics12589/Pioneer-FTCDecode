package org.firstinspires.ftc.teamcode.utility.autonomous;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PolyPath;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

public class PolyTrajectory {
    private ArrayList<PolyPath> paths;

    public PolyTrajectory() {
    }

    public void addPath(PolyPath path) {
        paths.add(path);
    }

    public void runTrajectory() {
        for (PolyPath path : paths) {
            path.runPath();
        }
    }
}