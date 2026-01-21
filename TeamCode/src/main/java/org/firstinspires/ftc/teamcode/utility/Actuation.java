package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.security.InvalidParameterException;

public class Actuation {
    private static boolean slowMode = false;
    private static boolean slowModeToggle = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotor intake, transfer;
    public static Servo blocker;
    public static DcMotorEx flywheel, turret, flywheel1, flywheel2;

    public static NormalizedColorSensor colorSensor;

    public static Telemetry telemetry;
    public static Limelight3A limelight;
    public static FtcDashboard dashboard;
    public static TelemetryPacket packet;

    public static void setup(HardwareMap map, Telemetry tel) {
        OttoCore.setup(map);

        telemetry = tel;

        if (map.dcMotor.contains("frontLeft")) {
            frontLeft = map.get(DcMotor.class, "frontLeft");
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (map.dcMotor.contains("frontRight")) {
            frontRight = map.get(DcMotor.class, "frontRight");
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.dcMotor.contains("backLeft")) {
            backLeft = map.get(DcMotor.class, "backLeft");
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (map.dcMotor.contains("backRight")) {
            backRight = map.get(DcMotor.class, "backRight");
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.dcMotor.contains("transfer")) {
            transfer = map.get(DcMotor.class, "transfer");
        }
        if (map.dcMotor.contains("intake")) {
            intake = map.get(DcMotor.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.servo.contains("blocker")) {
            blocker = map.get(Servo.class, "blocker");
        }

        if (map.dcMotor.contains("flywheel")) {
            flywheel = map.get(DcMotorEx.class, "flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
        }


//        if (map.dcMotor.contains("flywheel1")) {
//            flywheel1 = map.get(DcMotorEx.class, "flywheel1");
//            flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
//            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
//        }
//        if (map.dcMotor.contains("flywheel2")) {
//            flywheel2 = map.get(DcMotorEx.class, "flywheel2");
//            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
//            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
//        }

        if (map.dcMotor.contains("turret")) {
            turret = map.get(DcMotorEx.class, "turret");
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           // turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.turretPIDRot);
        }

        try {
            colorSensor = map.get(NormalizedColorSensor.class, "colorSensor");
        } catch (Exception e) {
            colorSensor = null;
        }


//        if (map.getAllNames(Limelight3A.class).contains("limelight")) {
//            limelight = map.get(Limelight3A.class, "limelight");
//            setupLimelight(0);
//        }

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    public static void drive(double move, double turn, double strafe) {
        frontLeft.setPower(move + turn + strafe);
        frontRight.setPower(move - turn - strafe);
        backLeft.setPower(move + turn - strafe);
        backRight.setPower(move - turn + strafe);
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
//        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
//        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
//        flywheel1.setVelocity(velocity);
//        flywheel2.setVelocity(velocity);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
        flywheel.setVelocity(velocity);
        packet.put("target vel", velocity);
        packet.put("actual vel", flywheel.getVelocity());
//        packet.put("actual vel", flywheel1.getVelocity());
//        packet.put("actual vel", flywheel2.getVelocity());
        updateTelemetry();
    }
    public static double getFlywheel() {
        return flywheel.getVelocity();
//        return flywheel1.getVelocity();
    }
    public static void checkFlywheelSpeed(Gamepad gamepad1, int targetVelocity) {
        if (Math.abs(flywheel.getVelocity() - targetVelocity) <= 20) {
            gamepad1.setLedColor(0, 1, 0, 100);
        } else {
            gamepad1.setLedColor(1, 0, 0, 100);
        }

        packet.put("flywheel velocity", flywheel.getVelocity());
        updateTelemetry();
    }
    public static void runIntake(boolean control) {
        if (control) {
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
            blocker.setPosition(0.5);
        }
        else {
            blocker.setPosition(0.0);
            intake.setPower(0.0);
        }
    }
    public static void runTransfer(boolean control, boolean shooting) {
        if (control) {
            if (!shooting) {
                if (!senseArtifact()) {
                    transfer.setPower(ActuationConstants.Intake.transferSpeed*0.6767);
                } else {
                    transfer.setPower(-0.1*ActuationConstants.Intake.transferSpeed*0.6767);
                    setFlywheel(-670);
                }
            } else {
                transfer.setPower(ActuationConstants.Intake.transferSpeed*0.6767);
            }
        }
        else {
            transfer.setPower(0.0);
        }
    }
    public static void runTransfer(boolean control, boolean shooting, double speed) {
        if (control) {
            if (!shooting) {
                if (!senseArtifact()) {
                    transfer.setPower(speed);
                } else {
                    transfer.setPower(-0.1 * speed);
                }
            } else {
                transfer.setPower(speed);
            }
        }
        else {
            transfer.setPower(0.0);
        }
    }
    public static double getTurret() {
        OttoCore.updatePosition();
        double turretAng = (double) turret.getCurrentPosition() / ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio + OttoCore.robotPose.heading;
        return turretAng % (2 * Math.PI);
    }

    /**
     * Rotates the turret to a global angle
     * @param angle global angle
     */
    public static void turretMoveTowards(double angle) {

        double ang_local = (angle - OttoCore.robotPose.heading + 2 * Math.PI) % (2 * Math.PI);

        double turretAngle = getTurret();
        double targetTicks = angle * (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio);

        if (turretAngle > ang_local) {
            while (Math.abs(turretAngle - ang_local) > Math.toRadians(180)) {
                ang_local += 2 * Math.PI;
            }
        } else if (turretAngle < ang_local) {
            while (Math.abs(turretAngle - ang_local) > Math.toRadians(180)) {
                ang_local -= 2 * Math.PI;
            }
        }

        // Spin if the target is further than the maximum angle for either rotation
        if (targetTicks > ActuationConstants.Launcher.turretMaxAngleTicks) {
            ang_local = -ActuationConstants.Launcher.turretMaxAngle;
        } else if (ang_local < -ActuationConstants.Launcher.turretMaxAngleTicks) {
            ang_local = ActuationConstants.Launcher.turretMaxAngle;
        }

        double turretSignal = ActuationConstants.Launcher.turretPIDRot.calculateSignal(ang_local, turretAngle);
        double clampedTurret = Math.max(-1.0, Math.min(1.0, turretSignal)); // Clamp signal between -1 & 1

        turret.setPower(clampedTurret);
    }
    public static void controlTurret(double value) {
        double turretAngle = getTurret();

        if (turretAngle > ActuationConstants.Launcher.turretMaxAngle) {
            turret.setPower(0);
        } else if (turretAngle < -ActuationConstants.Launcher.turretMaxAngle) {
            turret.setPower(0);
        } else {
            turret.setPower(value*0.25);
        }

    }
    public static void reverse(boolean control) {
        if (control) {
            transfer.setPower(-ActuationConstants.Intake.transferSpeed);
            intake.setPower(-ActuationConstants.Intake.intakeSpeed);
            flywheel.setVelocity(-670);
        }
    }
    public static boolean senseArtifact() {
        NormalizedRGBA sense = colorSensor.getNormalizedColors();
        int[] colors = new int[3];
        colors[0] = (int) (sense.red * 255);
        colors[1] = (int) (sense.green * 255);
        colors[2] = (int) (sense.blue * 255);

        float[] hsvValues = new float[3];
        Color.RGBToHSV(colors[0], colors[1], colors[2], hsvValues);

        telemetry.addData("Color Sensor", hsvValues[2]);

        return hsvValues[2] > 0.0;
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
    public static void updateTelemetry() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}

