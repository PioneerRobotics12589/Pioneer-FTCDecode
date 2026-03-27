package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.utility.ActuationConstants.Intake.blockerDown;
import static org.firstinspires.ftc.teamcode.utility.ActuationConstants.Intake.blockerUp;
import static org.firstinspires.ftc.teamcode.utility.ActuationConstants.Launcher.turretFF;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoLaunch;
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
    public static Servo blocker1, blocker2, launchIndicator1, launchIndicator2;
    public static DcMotorEx flywheel, flywheel1;
    private static int lastPosition = 0;
    private static long lastTime = 0;
    private static long lastBlockerTime = 0;
    private static double lastBlockerPos = 0;

    public static Telemetry telemetry;
    public static Limelight3A limelight;
    public static FtcDashboard dashboard;
    public static TelemetryPacket packet;

    public static void setup(HardwareMap map, Telemetry tel) {
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
        if (map.dcMotor.contains("transfer")) {
            transfer = map.get(DcMotor.class, "transfer");
            //transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.dcMotor.contains("intake")) {
            intake = map.get(DcMotor.class, "intake");
            //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.servo.contains("blocker1")) {
            blocker1 = map.get(Servo.class, "blocker1");
        }
        if (map.servo.contains("blocker2")) {
            blocker2 = map.get(Servo.class, "blocker2");
        }
        if (map.servo.contains("launchIndicator1")) {
            launchIndicator1 = map.get(Servo.class, "launchIndicator1");
        }
        if (map.servo.contains("launchIndicator2")) {
            launchIndicator2 = map.get(Servo.class, "launchIndicator2");
        }

        if (map.dcMotor.contains("flywheel1")) {
            flywheel = map.get(DcMotorEx.class, "flywheel1");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
            //flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.dcMotor.contains("flywheel2")) {
            flywheel1 = map.get(DcMotorEx.class, "flywheel2");
            flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
//            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
            //flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.dcMotor.contains("turret")) {
            turret = map.get(DcMotor.class, "turret");
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        frontLeft.setPower(voltageCompensation(move + turn + strafe));
        frontRight.setPower(voltageCompensation(move - turn - strafe));
        backLeft.setPower(voltageCompensation(move + turn - strafe));
        backRight.setPower(voltageCompensation(move - turn + strafe));
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

        double moveP = Math.abs(move) > 0.05 ? move : 0;
        double strafeP = Math.abs(strafe) > 0.05 ? strafe : 0;
        double turnP = Math.abs(turn) > 0.05 ? turn : 0;


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
//        velocity = Math.max(1330, velocity);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
//        double feedforward = ActuationConstants.Launcher.flywheelFF.calculate(velocity);
//        double pid = ActuationConstants.Launcher.flywheelPID.calculate(velocity);
//        double signal = Math.max(0, Math.min(1, voltageCompensation(feedforward + pid)));
//        flywheel.setPower(signal);
        flywheel.setVelocity(velocity);
        flywheel1.setVelocity(velocity);
    }

    /**
     * Determines the current flywheel velocity (Custom without flywheel.getVelocity() because it always returns 0 for some reason)
     * @return angular flywheel velocity in ticks per second
     */
    public static double getFlywheel() {
        int currentPosition = flywheel.getCurrentPosition();
        long currentTime = System.nanoTime();

        if (lastTime == 0) {
            lastTime = currentTime;
            lastPosition = currentPosition;
            return 0;
        }

        double deltaTicks = currentPosition - lastPosition;
        double deltaTime = (currentTime - lastTime) / 1e9;

        lastPosition = currentPosition;
        lastTime = currentTime;

        if (deltaTime == 0) return 0;

        return deltaTicks / deltaTime;
    }

    public static boolean flywheelIsReady(int targetVelocity) {
        return getFlywheel() >= targetVelocity-10;
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
     * @param control gamepad1 intake control
     */
    public static void runIntake(boolean control) {
        if (control) {
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
        }else {
            intake.setPower(0.0);
        }
    }
    public static void intake(boolean control) {
        if (control) {
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
        }else {
            intake.setPower(0.0);
        }
    }

    /**
     * Runs the intake at the pre-specified power
     * @param control1 gamepad1 intake control
     * @param control2 gamepad2 intake control
     */
    public static void runIntake(boolean control1, boolean control2) {
        if (control1 || control2) {
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
        } else {
            intake.setPower(0.0);
        }
    }

    public static void setBlocker(boolean control) {
        if (control) {
            blocker1.setPosition(blockerDown);
            blocker2.setPosition(blockerDown);
        } else {
            blocker1.setPosition(blockerUp);
            blocker2.setPosition(blockerUp);
        }
        if (lastBlockerPos != blocker1.getPosition()) {
            lastBlockerTime = System.currentTimeMillis();
        }
        lastBlockerPos = blocker1.getPosition();
    }

    public static boolean blockerAtPos(double position) {
        return (System.currentTimeMillis() - lastBlockerTime > 450 && blocker1.getPosition() == position);
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
            transfer.setPower(-ActuationConstants.Intake.transferSpeed);
        }
        else {
            transfer.setPower(0.0);
        }
    }

    /**
     * Reverses intake and transfer
     * @param control motor power
     */

    public static void shoot(boolean control) {
        if (control) {
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
            transfer.setPower(ActuationConstants.Intake.transferSpeed);
        } else {
            intake.setPower(0.0);
            transfer.setPower(0.0);
        }
    }

    public static void reverse(boolean control) {
        if (control) {
            transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.5);
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    public static void runTransferSlow(boolean control) {
        if (control) {
            transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.5);
        } else {
            transfer.setPower(0);
        }
    }

    /**
     * Sets the RGB indicator depending on if the robot is in the launch zone
     */
    public static void setLaunchIndicator(double time) {
        if (AutoLaunch.inLaunchZone() && AutoMovement.turretReady && AutoMovement.flywheelReady) {
            launchIndicator1.setPosition(1);


        } else {
            launchIndicator1.setPosition(0.0);
        }
        if (time >= 60.0 && time < 90.0) {
            launchIndicator2.setPosition(0.388);
        }
        else if (time >= 90.0 && time < 100.0) {
            launchIndicator2.setPosition(0.333);
        }
        else if (time >= 100.0 && time < 110.0) {
            launchIndicator2.setPosition(0.3);
        }
        else if (time >= 110.0) {
            launchIndicator2.setPosition(0.722);
        }
        else {
            launchIndicator2.setPosition(0.5);
        }
    }

    /**
     * Finds the global angle of the turret
     * @return turret's global angle
     */
    public static double getTurretGlobal() {
        return (double) turret.getCurrentPosition() / (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio) + OttoCore.robotPose.heading;
    }

    /**
     * Finds the local angle of the turret
     * @return turret's global angle
     */
    public static double getTurretLocal() {
        return (double) turret.getCurrentPosition() / (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio);
    }

    /**
     * Rotates the turret towards a global angle
     * @param target global angle
     */
    public static void turretMoveTowards(double target) {
        double targetLocal = AngleUnit.normalizeRadians(target - OttoCore.robotPose.heading);
        targetLocal = Math.max(-ActuationConstants.Launcher.turretMaxAngle, Math.min(ActuationConstants.Launcher.turretMaxAngle, targetLocal));
        double currentLocal = AngleUnit.normalizeRadians(getTurretLocal());

        double turretPID = ActuationConstants.Launcher.turretPID.calculateSignal(targetLocal, currentLocal);
//        double turretFF = (Math.abs(targetLocal - currentLocal) > Math.toRadians(0.5)) ? Math.signum(turretPID) * ActuationConstants.Launcher.turretFF : 0.0;
        turret.setPower(voltageCompensation(turretPID));

        telemetry.addData("Actuation: Current Local Angle", AngleUnit.normalizeRadians(getTurretLocal()));
        telemetry.addData("Actuation: Target Local Angle", targetLocal);
        telemetry.addData("Actuation: Current Global Angle", AngleUnit.normalizeRadians(getTurretGlobal()));
        telemetry.addData("Actuation: Target Global Angle", target);
    }

    public static void turretMoveTowardsLocal(double targetLocal) {
        double currentLocal = getTurretLocal();

        double turretPID = ActuationConstants.Launcher.turretPID.calculateSignal(targetLocal, currentLocal);
        turret.setPower(voltageCompensation(turretPID));
    }

    public static void setTurret(double angle) {
        while (Math.abs(AngleUnit.normalizeRadians(getTurretGlobal() - angle)) >= Math.toRadians(1)) {
            telemetry.addData("Turret Angle", Math.toDegrees(AngleUnit.normalizeRadians(getTurretGlobal())));
            telemetry.addData("Target Angle", angle);
            turretMoveTowards(angle);
        }
        turret.setPower(0);
    }

    public static double voltageCompensation(double power) {
        double multiplier = 13.0 / OttoCore.voltageSensor.getVoltage();
        return Math.min(1.0, Math.max(-1.0, power * multiplier));
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