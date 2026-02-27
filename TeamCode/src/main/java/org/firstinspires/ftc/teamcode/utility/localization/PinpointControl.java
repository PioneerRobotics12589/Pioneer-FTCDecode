package org.firstinspires.ftc.teamcode.utility.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class PinpointControl {
    private static GoBildaPinpointDriver pinpoint;
    public static double xMult = 1.004016064257, yMult = 1.00180863274672;
    private static Pose currentPose;
    private static Pose velocityPose;

    private static Pose poseOffset;
    
    /**
     * Pinpoint Odometry Program: <a href="https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint/blob/goBILDA-Odometry-Driver/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SensorGoBildaPinpointExample.java">...</a>
     * Pinpoint guide: <a href="https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf">...</a>
     * @param map Hardware map
     * @param xOffset X odometry pod offset
     * @param yOffset Y odometry pod offset
     */
    public static void setup(HardwareMap map, double xOffset, double yOffset) {
        pinpoint = map.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // ^ Directions or offsets can be a reason why position accumulates
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        currentPose = new Pose(0, 0, 0);
        velocityPose = new Pose(0, 0, 0);
        poseOffset = new Pose(0, 0, 0);
    }

    public static void resetPose() {
        pinpoint.resetPosAndIMU();
    }

    public static void setPose(Pose newPose) {
        updatePose();
        poseOffset = new Pose(newPose.x - currentPose.x, newPose.y - currentPose.y, AngleUnit.normalizeRadians(newPose.heading - currentPose.heading));
    }

    public static void updatePose() {
        pinpoint.update();
        double xOdo = pinpoint.getPosX(DistanceUnit.INCH);
        double yOdo = pinpoint.getPosY(DistanceUnit.INCH);
        double angle = 0.015067;
//        currentPose.x = (xOdo*Math.cos(angle) - yOdo*Math.sin(angle) + poseOffset.x) * xMult;
//        currentPose.y = (xOdo*Math.sin(angle) + yOdo*Math.cos(angle) + poseOffset.y) * yMult;
        currentPose.x = (xOdo + poseOffset.x) * xMult;
        currentPose.y = (yOdo + poseOffset.y) * yMult;
        currentPose.heading = AngleUnit.normalizeRadians(pinpoint.getHeading(AngleUnit.RADIANS) + poseOffset.heading);
    }

    public static Pose getPose() {
        return currentPose;
    }

    public static void updateVelocityPose() {
        velocityPose.x = pinpoint.getVelX(DistanceUnit.INCH);
        velocityPose.y = pinpoint.getVelY(DistanceUnit.INCH);
        velocityPose.heading = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public static Pose getVelocityPose() {
        return velocityPose;
    }
}
