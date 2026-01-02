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
            if (isMotif(fid)) {
                artifactPattern = fid.getFiducialId();
                return fid;
            }
        }
        return null;
    }

    /**
     * Determines the fiducial of the specified goal
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

    /**
     * Determines if a fiducial is on the Obelisk
     * @param fid fiducial
     * @return true: fiducial is the motif, false otherwise
     */
    public static boolean isMotif(LLResultTypes.FiducialResult fid) {
        int id = fid.getFiducialId();
        return id == 21 || id == 22 || id == 23;
    }

    /**
     * Determines the global position of the robot based on fiducial positions
     * @param fiducial located fiducial
     * @return Estimated global robot position
     */
    public static double[] getGlobalPosition(LLResultTypes.FiducialResult fiducial) {
        // Jayden, you got this...
        // No I don't bro := (
        return new double[0];
    }

    /**
     * Updates the odometry based on the new estimated positions by the fiducials
     */
    public static void visualOdometryUpdate() {
        Pose newRobotPose = new Pose(0, 0, OttoCore.robotPose.heading);

        int fidCount = 0;
        for (LLResultTypes.FiducialResult fid : getFiducials()) {
            if (!isMotif(fid)) {
                double[] posRel = getGlobalPosition(fid);
                newRobotPose.x += posRel[0];
                newRobotPose.y += posRel[1];
                fidCount++;
            }
        }
        if (fidCount != 0) {
            OttoCore.robotPose = new Pose(newRobotPose.x / fidCount, newRobotPose.y / fidCount, OttoCore.robotPose.heading);
        }
    }
}
