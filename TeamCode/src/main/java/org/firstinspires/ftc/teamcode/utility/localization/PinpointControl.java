

public class PinpointControl {
    private static GoBildaPinpointDriver pinpoint;

    private static Pose currentPose;
    private static Pose velocityPose;

    private static Pose poseOffset;
    
    /**
     * Pinpoint Odometry Program: https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint/blob/goBILDA-Odometry-Driver/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SensorGoBildaPinpointExample.java
     * 
     */
    public static void setup(HardwareMap map, double xOffset, double yOffset) {
        pinpoint = map.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GOBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public static void resetPose() {
        pinpoint.resetPosAndIMU();
    }

    public static void setPose(Pose newPose) {
        resetPose();
        poseOffset = newPose;
    }

    public static void updatePose() {
        currentPose.x = pinpoint.getX(DistanceUnit.INCH) + poseOffset.x;
        currentPose.y = pinpoint.getY(DistanceUnit.INCH) + poseOffset.y;
        currentPose.heading = pinpoint.getHeading(DistanceUnit.RADIANS) + poseOffset.heading;
    }

    public static void getPose() {
        return currentPose;
    }

    public static void updateVelocityPose() {
        velocityPose.x = pinpoint.getVelX(DistanceUnit.INCH);
        velocityPose.y = pinpoint.getVelY(DistanceUnit.INCH);
        velocityPose.heading = pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }
}
