package org.firstinspires.ftc.teamcode.utility.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class PinpointControl {
    private static GoBildaPinpointDriver pinpoint;
    private static Pose currentPose;
    private static Pose lastPose;
    private static Pose velocityPose;
    private static Pose accelerationPose;
    private static long lastTime;
    private static double deltaX, deltaY;
    private final static double xMult = 1.0, yMult = -1.0;

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
        lastPose = new Pose(0, 0, 0);
        velocityPose = new Pose(0, 0, 0);
        accelerationPose = new Pose(0, 0, 0);

        lastTime = System.nanoTime();
    }

    public static void setOffsets(double xOffset, double yOffset) {
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
    }

    public static void resetPose() {
        pinpoint.resetPosAndIMU();
    }

    public static void setPose(Pose newPose) {
        updatePose();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, newPose.x, newPose.y, AngleUnit.RADIANS, newPose.heading));
        pinpoint.update();
        lastPose = new Pose(newPose);
    }

    public static void updatePose() {
        pinpoint.update();
        double xOdo = pinpoint.getPosX(DistanceUnit.INCH);
        double yOdo = pinpoint.getPosY(DistanceUnit.INCH);
        double angle = 0.015067;
        deltaX = currentPose.x - lastPose.x;
        deltaY = currentPose.y - lastPose.y;
        currentPose.x = xOdo + (deltaX*Math.cos(angle) - deltaY*Math.sin(angle)) * xMult;
        currentPose.y = yOdo + (deltaX*Math.sin(angle) + deltaY*Math.cos(angle)) * yMult;
        lastPose = new Pose(currentPose);
        currentPose.heading = AngleUnit.normalizeRadians(pinpoint.getHeading(AngleUnit.RADIANS));
    }

    public static Pose getPose() {
        return currentPose;
    }

    public static void updateVelocityPose() {
        pinpoint.update();
        velocityPose.x = pinpoint.getVelX(DistanceUnit.INCH);
        velocityPose.y = pinpoint.getVelY(DistanceUnit.INCH);
        velocityPose.heading = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public static Pose getVelocityPose() {
        return velocityPose;
    }

    public static void updateAccelerationPose() {
        Pose prevVel = new Pose(velocityPose);
        updateVelocityPose();
        double dt = (System.nanoTime() - lastTime)/1000000000.00;

        accelerationPose.x = (velocityPose.x - prevVel.x) / dt;
        accelerationPose.y = (velocityPose.y - prevVel.y) / dt;
        accelerationPose.heading = (velocityPose.heading - prevVel.heading) / dt;
    }

    public static Pose getAccelerationPose() {
        return accelerationPose;
    }
}
