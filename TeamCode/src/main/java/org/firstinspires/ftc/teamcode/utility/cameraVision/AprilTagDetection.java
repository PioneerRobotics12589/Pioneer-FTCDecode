package org.firstinspires.ftc.teamcode.utility.cameraVision;

import com.qualcomm.hardware.limelightvision.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;


import java.util.List;

public class AprilTagDetection {
    private static int artifactPattern;
    private String team;

    public static List<LLResultTypes.FiducialResult> getFiducials() {
        Actuation.setPipeline(ActuationConstants.LimelightConsts.PIPELINE_APRILTAG);
        return Actuation.getLLResult().getFiducialResults();
    }

    public static LLResult getResult(){
        return Actuation.getLLResult();
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

    public static double getTx(LLResult result) {
        double tx = Double.NaN;
//        for (LLResultTypes.FiducialResult fid : fiducials) {
//            if (fid.getFiducialId() == targetId) {
//                tx = Math.toRadians(fid.getTargetXDegrees());
//                // Auto-adjusts robot position based on AprilTag angle;
////                double global_angle = Actuation.getTurretGlobal()-tx;
////                double m = Math.tan(global_angle), b = tag.y - tag.x*m;
////                double new_x = (OttoCore.robotPose.x - m*b + m*OttoCore.robotPose.y) / (Math.pow(m, 2) + 1);
////                OttoCore.robotPose = OttoCore.relativeTransform(new Pose(new_x, m*new_x+b, OttoCore.robotPose.heading), -ActuationConstants.Launcher.turretOffset, 0, 0);
////                reference = OttoCore.relativeTransform(fiducialGlobalPos, ActuationConstants.Launcher.turretOffset, 0, 0);
//            }
//        }
        tx = Math.toRadians(result.getTx());
        return tx;
    }

    public static Point getTagGlobalPos(List<LLResultTypes.FiducialResult> fiducials) {
        Pose myGlobalPose = getGlobalPos(fiducials);
        int numberOfFids = 0;
        double sumX = 0, sumY = 0;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            if (id != 20 && id != 24) continue;
            sumX += fiducial.getTargetPoseRobotSpace().getPosition().x * 39.3701;
            sumY += fiducial.getTargetPoseRobotSpace().getPosition().y * 39.3701;
            numberOfFids++;
        }
        Pose LocalTagPose = new Pose(sumX/numberOfFids, sumY/numberOfFids, 0.0);
        double myHeading = OttoCore.robotPose.heading;
        return new Point(LocalTagPose.x * Math.cos(myHeading) + LocalTagPose.y * Math.sin(myHeading) + myGlobalPose.x, LocalTagPose.y * Math.cos(myHeading) - LocalTagPose.x * Math.sin(myHeading) + myGlobalPose.y);
    }

    public static Pose getGlobalPos(List<LLResultTypes.FiducialResult> fiducials) {
        // Jayden, you got this...
//        double[] position = {fiducial.getRobotPoseFieldSpace().getPosition().x * 39.3701, fiducial.getRobotPoseFieldSpace().getPosition().y * 39.3701};
        double sumX = 0, sumY = 0, sumH = 0;
        int numberOfFids = 0;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            if (id != 20 && id != 24 && validDistance(fiducial, 100)) continue;
            sumX += fiducial.getRobotPoseFieldSpace().getPosition().x * 39.3701;
            sumY += fiducial.getRobotPoseFieldSpace().getPosition().y * 39.3701;
            sumH += (fiducial.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.RADIANS) + 2 * Math.PI) % (2 * Math.PI);
            numberOfFids++;
        }
        if (numberOfFids == 0) return null;

        return new Pose(sumX/numberOfFids, sumY/numberOfFids, sumH/numberOfFids);
    }

    private static boolean validDistance(LLResultTypes.FiducialResult fid, double maxDist) {
        double relX = fid.getRobotPoseTargetSpace().getPosition().x * 39.3701;
        double relY = fid.getRobotPoseTargetSpace().getPosition().y * 39.3701;
        return Math.hypot(relX, relY) < maxDist;
    }
}
