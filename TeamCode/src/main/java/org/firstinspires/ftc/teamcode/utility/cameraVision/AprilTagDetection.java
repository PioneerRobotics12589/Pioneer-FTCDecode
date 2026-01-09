package org.firstinspires.ftc.teamcode.utility.cameraVision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

import java.util.List;

public class AprilTagDetection {
    private static int artifactPattern;
    private static String team;

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

    public static double[] getGlobalPosition(LLResultTypes.FiducialResult fiducial) {
        // Jayden, you got this...
        double[] position = {fiducial.getRobotPoseFieldSpace().getPosition().x * 39.3701, fiducial.getRobotPoseFieldSpace().getPosition().y * 39.3701};
        return position;
    }
}
