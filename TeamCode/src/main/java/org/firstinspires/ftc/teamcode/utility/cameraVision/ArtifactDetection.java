package org.firstinspires.ftc.teamcode.utility.cameraVision;

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

        // Center pixel of contour (0, 0) is the center of the image
        double pixelX = contour[0] - ActuationConstants.LimelightConsts.RESOLUTION_X / 2.0, pixelY = contour[1] - ActuationConstants.LimelightConsts.RESOLUTION_Y / 2.0;

        return pixelX;
    }

    /**
     * Finds the turn value to track the closest artifact
     * @param color color of desired artifact
     * @return turn value to track artifact
     */
    public static double trackArtifact(String color) {
        double pixelX = locateArtifact(color); // X-coordinate of contour center

        OttoCore.updatePosition();
        double head_signal = ActuationConstants.LimelightConsts.head_PID.calculateSignal(pixelX, OttoCore.robotPose.heading); // PID Signal

        head_signal = Math.max(-1, Math.min(head_signal, 1)); // Clamp

        return head_signal; // Turn power to track artifact
    }
}