package org.firstinspires.ftc.teamcode.utility.cameraVision;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.opencv.core.MatOfPoint;

import java.security.InvalidParameterException;

public class ArtifactDetection {
    private static int pattern;

    public static void setPattern(int motifID) {
        pattern = motifID;
    }

    public static void locateArtifact(String team) {
        if (team.equals("green")) {
            Actuation.setPipeline(0);
        } else if (team.equals("purple")) {
            Actuation.setPipeline(1);
        } else {
            throw new InvalidParameterException("ArtifactDetection.localArtifact(): Invalid Team");
        }

        LLResult pysnapResult = Actuation.getLLResult();

        double[] contour = pysnapResult.getPythonOutput(); // Coordinates of largest contour

        double pixelX = contour[0], pixelY = contour[1];


    }
}
