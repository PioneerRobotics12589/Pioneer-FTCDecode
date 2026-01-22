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

    public static DcMotor intake, transfer, turret;
    public static Servo blocker;
    public static DcMotorEx flywheel, flywheel1, flywheel2;
    public static int turretInitTicks; // Needed to "zero" the turret

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

        if (map.dcMotor.contains("turret")) {
            turret = map.get(DcMotor.class, "turret");
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretInitTicks = turret.getCurrentPosition();
        }

        if (map.getAllNames(Limelight3A.class).contains("limelight")) {
            limelight = map.get(Limelight3A.class, "limelight");
            setupLimelight(0);
        }

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    /**
     * Mecanum wheel drive
     * @param move move power
     * @param turn turn power
     * @param strafe strafe power
     */
    public static void drive(double move, double turn, double strafe) {
        frontLeft.setPower(move + turn + strafe);
        frontRight.setPower(move - turn - strafe);
        backLeft.setPower(move + turn - strafe);
        backRight.setPower(move - turn + strafe);
    }

    /**
     * Mecanum wheel drive
     * @param toggleSlowMode slow mode (on/off)
     * @param move move power
     * @param turn turn power
     * @param strafe strafe power
     */
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

    /**
     * Uses a PID system to set the flywheel to a specified velocity
     * @param velocity target flywheel angular velocity
     */
    public static void setFlywheel(int velocity) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
        flywheel.setVelocity(velocity);
    }

    /**
     * Determines the current flywheel velocity
     * @return angular flywheel velocity
     */
    public static double getFlywheel() {
        return flywheel.getVelocity();
    }

    /**
     * Changes the gamepad color depending on the flywheel speed
     * @param gamepad1 gamepad
     * @param targetVelocity target flywheel velocity
     */
    public static void checkFlywheelSpeed(Gamepad gamepad1, int targetVelocity) {
        if (Math.abs(getFlywheel() - targetVelocity) <= 20) {
            gamepad1.setLedColor(0, 1, 0, 100);
        } else {
            gamepad1.setLedColor(1, 0, 0, 100);
        }
    }

    /**
     * Runs the intake at the pre-specified power
     * @param control on/off
     */
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

    /**
     * Runs the transfer at the specified power
     * @param control on/off
     * @param speed transfer speed
     */
    public static void runTransfer(boolean control, double speed) {
        if (control) {
            transfer.setPower(speed);
        }
        else {
            transfer.setPower(0.0);
        }
    }

    /**
     * Runs the transfer at the pre-specified power
     * @param control on/off
     */
    public static void runTransfer(boolean control) {
        if (control) {
            transfer.setPower(ActuationConstants.Intake.transferSpeed);
        }
        else {
            transfer.setPower(0.0);
        }
    }

    /**
     * Reverses intake and transfer
     * @param control motor power
     */
    public static void reverse(boolean control) {
        if (control) {
            transfer.setPower(-ActuationConstants.Intake.transferSpeed);
            intake.setPower(-ActuationConstants.Intake.intakeSpeed);
            flywheel.setVelocity(-670);
        }
    }
    /**
     * Finds the global angle of the turret
     * @return turret's global angle
     */
    public static double getTurret() {
        OttoCore.updatePosition();
        double turretAng = (double) (turret.getCurrentPosition() - turretInitTicks) / ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio + OttoCore.robotPose.heading;
        return turretAng % (2 * Math.PI);
    }

    /**
     * Rotates the turret towards a global angle
     * @param angle global angle
     */
    public static void turretMoveTowards(double angle) {

        double ang_local = (angle - OttoCore.robotPose.heading + 2 * Math.PI) % (2 * Math.PI);

        double turretAngle = getTurret();

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
        if (ang_local > ActuationConstants.Launcher.turretMaxAngle) {
            ang_local = -ActuationConstants.Launcher.turretMaxAngle;
        } else if (ang_local < -ActuationConstants.Launcher.turretMaxAngle) {
            ang_local = ActuationConstants.Launcher.turretMaxAngle;
        }

        double turretSignal = ActuationConstants.Launcher.turretPIDRot.calculateSignal(ang_local, turretAngle);
        double clampedTurret = Math.max(-1.0, Math.min(1.0, turretSignal)); // Clamp signal between -1 & 1

        turret.setPower(clampedTurret);
    }

    /**
     * Sets the turret motor value directly
     * @param value motor power
     */
    public static void controlTurret(double value) {
        double turretAngle = getTurret();

        if (turretAngle > ActuationConstants.Launcher.turretMaxAngle) {
            turret.setPower(0);
        } else if (turretAngle < -ActuationConstants.Launcher.turretMaxAngle) {
            turret.setPower(0);
        } else {
            turret.setPower(value*0.05);
        }
    }

    /**
     * Sets up the Limelight
     * @param pipeline initial pipeline number
     */
    public static void setupLimelight(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    /**
     * Sets the pipeline of the Limelight
     * @param pipeline pipeline number
     */
    public static void setPipeline(int pipeline) {
        /*
        / Pipeline  Function
        /    0      AprilTag Detection
        /    1      Green Artifact Detection
        /    2      Purple Artifact Detection
         */

        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Gets the result of the Limelight
     * @return Limelight result
     */
    public static LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    /**
     * Updates the telemetry packet
     */
    public static void updateTelemetry() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}

