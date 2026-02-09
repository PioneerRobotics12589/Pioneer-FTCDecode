package org.firstinspires.ftc.teamcode.utility.cameraVision;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.opencv.core.MatOfPoint;

import java.security.InvalidParameterException;

public class ArtifactDetection {

    /**
     * Finds the tx value for the largest artifact of the desired color
     * @return estimated pose needed to intake of the closest artifact
     */
    public static Pose locateArtifact() {
        // Switch pipeline to track desired color
        Actuation.setPipeline(ActuationConstants.LimelightConsts.PIPELINE_GREEN);

        LLResult pysnapResult = Actuation.getLLResult(); // Limelight result
        double[] output = pysnapResult.getPythonOutput();
        double tx_g = output[0];
        double ty_g = output[1];
        double rad_g = output[2];

        Actuation.setPipeline(ActuationConstants.LimelightConsts.PIPELINE_PURPLE);

        pysnapResult = Actuation.getLLResult(); // Limelight result
        output = pysnapResult.getPythonOutput();
        double tx_p = output[0];
        double ty_p = output[1];
        double rad_p = output[2];

        Pose robot = new Pose(OttoCore.robotPose);

        double h = ActuationConstants.LimelightConsts.limelightHeight - 2*ActuationConstants.Launcher.artifactRadius*39.37;
        double d_for, d_lat;
        Pose artifact;

        if (Math.abs(ty_p) > Math.abs(ty_g)) {
            // purple is closer
            d_for = h / Math.tan(Math.toRadians(ty_p));
            d_lat = d_for / Math.tan(Math.toRadians(tx_p));
        } else {
            // green is closer
            d_for = h / Math.tan(Math.toRadians(ty_g));
            d_lat = d_for / Math.tan(Math.toRadians(tx_g));
        }

        artifact = OttoCore.relativeTransform(robot, d_for, d_lat, 0);

        return artifact; // Largest contour tx value
    }

    /**
     * Moves towards the closest artifact
     * @return if the artifact has been intaken
     */
    public static boolean goToArtifact() {
        Pose artifact = locateArtifact(); // Artifact Pose

        OttoCore.updatePosition();
        OttoCore.moveTowards(artifact, 0.5, 0.5); // Go to artifact Position

        return OttoCore.robotPose.withinRange(artifact, 0.2, 0.2, Math.toRadians(360));
    }
}