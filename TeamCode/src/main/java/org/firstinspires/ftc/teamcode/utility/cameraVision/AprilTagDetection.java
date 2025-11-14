package org.firstinspires.ftc.teamcode.utility.cameraVision;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetection {

    private static LLResult result;
    private static int artifactPattern;
    private static String team;

    public static List<LLResultTypes.FiducialResult> getFiducials() {
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return null;
    }

    /**
     * Determines the pattern on the obelisk
     * @param fiducials list of fiducials detected in the image
     * @return motif fiducial (21=GPP 22=PGP 23=PPG)
     */
    public static LLResultTypes.FiducialResult getMotif(List<LLResultTypes.FiducialResult> fiducials) {
        for (LLResultTypes.FiducialResult fid : fiducials) {
            int id = fid.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                artifactPattern = id;
                return fid;
            }
        }
        return null;
    }

    /**
     * Determines the motif of the specified goal
     * @param fiducials list of fiducials detected in the image
     * @return motif fiducial (21=GPP 22=PGP 23=PPG)
     */
    public static LLResultTypes.FiducialResult getGoal(List<LLResultTypes.FiducialResult> fiducials) {
        for (LLResultTypes.FiducialResult fid : fiducials) {
            int id = fid.getFiducialId();
            if (id == 20 && team.equals("red") || id == 24 && team.equals("blue")) {
                return fid;
            }
        }
        return null;
    }

    public static boolean isPatternDetected() {
        return artifactPattern != -1;
    }

    public static void setTeam(String newTeam) {
        team = newTeam;
    }

    public static double[] getFiducialPosition(LLResultTypes.FiducialResult fiducial) {
        // Jayden, you got this...
        return new double[0];
    }
}
