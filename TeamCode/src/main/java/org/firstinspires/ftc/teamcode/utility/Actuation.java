package org.firstinspires.ftc.teamcode.utility;

import android.graphics.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.io.InvalidObjectException;
import java.security.InvalidParameterException;

public class Actuation {
    public static boolean slowMode = false;
    private static boolean slowModeToggle = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight, leftDrive, rightDrive;

    public static DcMotorEx flywheel;

    public static CRServo leftLoader, rightLoader;

    public static Telemetry telemetry;
    public static Limelight3A limelight;
    public static FtcDashboard dashboard;
    public static TelemetryPacket packet;

    public static void setup(HardwareMap map, Telemetry tel) {
        OttoCore.setup(map);

        telemetry = tel;

        if (map.dcMotor.contains("frontLeft")) {
            frontLeft = map.get(DcMotor.class, "frontLeft");
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (map.dcMotor.contains("frontRight")) {
            frontRight = map.get(DcMotor.class, "frontRight");
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.dcMotor.contains("backLeft")) {
            backLeft = map.get(DcMotor.class, "backLeft");
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (map.dcMotor.contains("backRight")) {
            backRight = map.get(DcMotor.class, "backRight");
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.dcMotor.contains("flywheel")) {
            flywheel = map.get(DcMotorEx.class, "flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.pidCoeffs);
        }

        if (map.crservo.contains("leftLoader")) {
            leftLoader = map.get(CRServo.class, "leftLoader");
            leftLoader.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.crservo.contains("rightLoader")) {
            rightLoader = map.get(CRServo.class, "rightLoader");
        }

        if (map.getAllNames(Limelight3A.class).contains("limelight")) {
            limelight = map.get(Limelight3A.class, "limelight");
        }

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    public static void setupStarter(HardwareMap map, Telemetry tel) {
        OttoCore.setup(map);

        telemetry = tel;

        if (map.dcMotor.contains("leftDrive")) {
            leftDrive = map.get(DcMotor.class, "leftDrive");
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (map.dcMotor.contains("rightDrive")) {
            rightDrive = map.get(DcMotor.class, "rightDrive");
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (map.dcMotor.contains("flywheel")) {
            flywheel = map.get(DcMotorEx.class, "flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.pidCoeffs);
        }

        if (map.crservo.contains("leftLoader")) {
            leftLoader = map.get(CRServo.class, "leftLoader");
            leftLoader.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.crservo.contains("rightLoader")) {
            rightLoader = map.get(CRServo.class, "rightLoader");
        }

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    public static void drive(double move, double turn, double strafe) {
        frontLeft.setPower(move + turn + strafe);
        frontRight.setPower(move - turn - strafe);
        backLeft.setPower(move + turn - strafe);
        backRight.setPower(move - turn + strafe);
    }

    public static void driveStarter(double move, double turn) {
        leftDrive.setPower(move + turn);
        rightDrive.setPower(move - turn);
    }

    public static void teleDrive(boolean toggleSlowMode, double move, double turn, double strafe) {
        if (toggleSlowMode && !slowModeToggle) slowMode = !slowMode;

        double moveP = move > 0.05 ? move : 0;
        double strafeP = strafe > 0.05 ? strafe : 0;
        double turnP = turn > 0.05 ? strafe : 0;


        double multip = (slowMode) ? 0.5 : 1.0;

        frontLeft.setPower((moveP+strafeP+turnP) * multip);
        backLeft.setPower((moveP-strafeP+turnP) * multip);
        frontRight.setPower((moveP-strafeP-turnP) * multip);
        backRight.setPower((moveP+strafeP-turnP) * multip);

        slowModeToggle = toggleSlowMode;
    }

    public static void setFlywheel(int velocity) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.pidCoeffs);
        flywheel.setVelocity(velocity);
        packet.put("target vel", velocity);
        packet.put("actual vel", flywheel.getVelocity());
        updateTelemetry();
    }

    public static void checkFlywheelSpeed(Gamepad gamepad1, int targetVelocity) {
        if (Math.abs(flywheel.getVelocity() - targetVelocity) <= 20) {
            gamepad1.setLedColor(0, 1, 0, 100);
        } else {
            gamepad1.setLedColor(1, 0, 0, 100);
        }
    }
    public static void setLoaders(boolean control) {
        if (control) {
            leftLoader.setPower(1.0);
            rightLoader.setPower(1.0);
        }
        else {
            leftLoader.setPower(0.0);
            rightLoader.setPower(0.0);
        }
    }
    public static void setupLimelight(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public static void setPipeline(int pipeline) {
        /*
        / Pipeline  Function
        /    0      AprilTag Detection
        /    1      Green Artifact Detection
        /    2      Purple Artifact Detection
         */

        limelight.pipelineSwitch(pipeline);
    }

    public static LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    public static double[] launchVals(String team) {
        Point goal = null;
        if (team.equals("red")) {
            goal = new Point(-72, 72);
        } else if (team.equals("blue")) {
            goal = new Point(72, 72);
        } else {
            throw new InvalidParameterException("Actuation.launchVals(): Invalid Team");
        }

        OttoCore.updatePosition();
        Pose pos = new Pose(OttoCore.robotPose);

        double dist = Math.sqrt(Math.pow(pos.x - goal.x, 2) + Math.pow(pos.y - goal.y, 2));
        double angle = Math.atan2(pos.x-goal.x, pos.y-goal.y);
        double height = ActuationConstants.Launcher.targetHeight + ActuationConstants.Launcher.artifactRadius - ActuationConstants.Drivetrain.launcherHeight;

//        double flywheelAngle = 0.5*Math.atan(-dist/height) + Math.PI/2.0;
        double flywheelAngle = 55.0 * Math.PI/180;
        double linVel = Math.sqrt(-9.8*Math.pow(dist, 2.0) / ((height-dist*Math.tan(flywheelAngle))*(2.0*Math.pow(Math.cos(flywheelAngle), 2.0))));

        double angVel = linVel / (ActuationConstants.Drivetrain.flwheelRad + ActuationConstants.Launcher.artifactRadius) * 180.0 / Math.PI;

        return new double[] {angle, angVel, flywheelAngle};
    }

    public static void updateTelemetry() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}

