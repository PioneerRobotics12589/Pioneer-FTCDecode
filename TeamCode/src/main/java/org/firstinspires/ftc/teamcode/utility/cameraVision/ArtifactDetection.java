package org.firstinspires.ftc.teamcode.utility.cameraVision;

import static org.firstinspires.ftc.teamcode.utility.Actuation.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.opencv.core.MatOfPoint;

import java.security.InvalidParameterException;

public class ArtifactDetection {

    /**
     * Finds the x-coordinate of the center of the closest artifact in front of the robot
     * @param color color of desired artifact
     * @return x-coordinate of artifact center
     */
    public static double locateArtifact(String color) {
        // Switch pipeline to track desired color
        if (color.equals("green")) {
            Actuation.setPipeline(ActuationConstants.LimelightConsts.PIPELINE_GREEN);
        } else if (color.equals("purple")) {
            Actuation.setPipeline(ActuationConstants.LimelightConsts.PIPELINE_PURPLE);
        } else {
            throw new InvalidParameterException("ArtifactDetection.localArtifact(): Invalid Team");
        }

        LLResult pysnapResult = Actuation.getLLResult(); // Limelight result

        double[] contour = pysnapResult.getPythonOutput(); // Center coordinates of largest contour

        return contour[1] - ActuationConstants.LimelightConsts.RESOLUTION_X / 2.0; // Displacement from artifact coordinates to image center
    }

    /**
     * Finds the turn value to track the closest artifact
     * @param color color of desired artifact
     * @return turn value to track artifact
     */
    public static double trackArtifact(String color) {
        double pixelX = locateArtifact(color); // Displacement from artifact contour X-coordinate to the center of image

        OttoCore.updatePosition();

        double head_signal = 0;
        int avg_count = 10;
        for (int i = 0; i < avg_count; i++) {
            head_signal += ActuationConstants.LimelightConsts.head_PID.calculateSignal(0, pixelX); // PID Signal
        }
        head_signal /= avg_count;

        head_signal = Math.max(-1, Math.min(head_signal, 1)); // Clamp

        return head_signal; // Turn power to track artifact
    }
}