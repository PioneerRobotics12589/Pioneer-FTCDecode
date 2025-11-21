package org.firstinspires.ftc.teamcode.utility.autonomous;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.cameraVision.ArtifactDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.PolyPath;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

public class PolyTrajectory {
    private final ArrayList<PolyPath> paths;

    public PolyTrajectory() {
        paths = new ArrayList<PolyPath>();
    }

    /**
     * Adds a path to the trajectory
     * @param path path to be added
     */
    public void addPath(PolyPath path) {
        paths.add(path);
    }

    /**
     * Runs the trajectory of paths
     */
    public void runTrajectory() {
        for (PolyPath path : paths) {
            path.runPath();
        }
    }
}