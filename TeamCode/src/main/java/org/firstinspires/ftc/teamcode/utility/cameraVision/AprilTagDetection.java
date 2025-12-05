package org.firstinspires.ftc.teamcode.utility.cameraVision;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AprilTagDetection {
    private static int motif = -1;
    public static final int GOAL_BLUE = 20, GOAL_RED = 24;
    public static final int MOTIF_GPP = 21, MOTIF_PGP = 22, MOTIF_PPG = 23;

    /**
     * Updates the image and determines fiducials
     * @return list of fiducials
     */
    public static List<LLResultTypes.FiducialResult> getFiducials() {
        Actuation.setPipeline(ActuationConstants.LimelightConsts.PIPELINE_APRILTAG);
        return Actuation.getLLResult().getFiducialResults();
    }

    /**
     * Determines the pattern on the obelisk
     * @param fiducials list of fiducials detected in the image
     * @return motif fiducial (21=GPP 22=PGP 23=PPG)
     */
    public static LLResultTypes.FiducialResult getMotif(List<LLResultTypes.FiducialResult> fiducials) {
        for (LLResultTypes.FiducialResult fid : fiducials) {
            int id = fid.getFiducialId();
            if (id == MOTIF_GPP || id == MOTIF_PGP || id == MOTIF_PPG) {
                motif = id;
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
    public static LLResultTypes.FiducialResult getGoal(List<LLResultTypes.FiducialResult> fiducials, String team) {
        for (LLResultTypes.FiducialResult fid : fiducials) {
            int id = fid.getFiducialId();
            if (id == GOAL_RED && team.equals("red") || id == GOAL_BLUE && team.equals("blue")) {
                return fid;
            }
        }
        return null;
    }

    /**
     * Determines if the obelisk's fiducial was detected
     * @return true: detected; false: undetected
     */
    public static boolean isPatternDetected() {
        return motif != -1;
    }

    /**
     * Determines the position of the fiducial relative to the field
     * @param fiducial fiducial who's position is to be detected
     * @return fiducial position
     */
    public static double[] getFiducialPosition(LLResultTypes.FiducialResult fiducial) {
        // Jayden, you got this...
        return new double[0];
    }
}