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
    public static Servo blocker, launchIndicator;
    public static DcMotorEx flywheel, flywheel1;

    public static Telemetry telemetry;
    public static Limelight3A limelight;
    public static FtcDashboard dashboard;
    public static TelemetryPacket packet;

    public static void setup(HardwareMap map, Telemetry tel) {
        telemetry = tel;
        OttoCore.setup(map);

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

        if (map.servo.contains("blocker")) {
            blocker = map.get(Servo.class, "blocker");
        }

        if (map.servo.contains("launchIndicator")) {
            launchIndicator = map.get(Servo.class, "launchIndicator");
        }

        if (map.dcMotor.contains("flywheel")) {
            flywheel = map.get(DcMotorEx.class, "flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ActuationConstants.Launcher.flywheelPID);
            //flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.dcMotor.contains("turret")) {
            turret = map.get(DcMotor.class, "turret");
            turret.setTargetPosition(0);
            turret.setPower(1); // Comment if using turret PID
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
     * @param control gamepad1 intake control
     */
    public static void runIntake(boolean control) {
        if (control) {
            intake.setPower(-ActuationConstants.Intake.intakeSpeed);
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
        if (control1) {
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
//            setBlocker(true);
        } else if (control2) {
            intake.setPower(ActuationConstants.Intake.intakeSpeed);
        } else {
//            setBlocker(false);
            intake.setPower(0.0);
        }
    }

    public static void setBlocker(boolean control) {
        if (control) {
            blocker.setPosition(ActuationConstants.Intake.blockerDown);
        } else {
            blocker.setPosition(ActuationConstants.Intake.blockerUp);
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
            //blocker.setPosition(ActuationConstants.Intake.blockerDown);
        } else {
            //blocker.setPosition(ActuationConstants.Intake.blockerUp);
            intake.setPower(0.0);
            transfer.setPower(0.0);
        }
    }

    public static void reverse(boolean control) {
        if (control) {
            transfer.setPower(-ActuationConstants.Intake.transferSpeed * 0.2);
            intake.setPower(-ActuationConstants.Intake.intakeSpeed);
        } else {
            intake.setPower(0);
            //flywheel.setVelocity(-670);
            //blocker.setPosition(ActuationConstants.Intake.blockerDown);
        } /*else {
            //blocker.setPosition(ActuationConstants.Intake.blockerUp);
            intake.setPower(0.0);
            transfer.setPower(0.0);
        }*/
    }

    /**
     * Sets the RGB indicator depending on if the robot is in the launch zone
     */
    public static void setLaunchIndicator() {
        if (AutoLaunch.inLaunchZone()) {
            launchIndicator.setPosition(1.0);
        } else {
            launchIndicator.setPosition(0.0);
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
        double targetLocal = AngleUnit.normalizeRadians(target - AngleUnit.normalizeRadians(OttoCore.robotPose.heading));
        targetLocal = Math.max(-ActuationConstants.Launcher.turretMaxAngle, Math.min(ActuationConstants.Launcher.turretMaxAngle, targetLocal));

//        double turretSignal = ActuationConstants.Launcher.turretPID.calculateSignal(targetLocal, AngleUnit.normalizeRadians(getTurretLocal()));
        int targetTicks = (int) (targetLocal * (ActuationConstants.Launcher.turretTicks * ActuationConstants.Launcher.turretRatio));
        turret.setTargetPosition(targetTicks);
//        turret.setPower(turretSignal);
    }

    /**
     * Sets the turret motor value directly
     * @param value motor power
     */
    public static void controlTurret(int value, double power) {
        double turretAngle = AngleUnit.normalizeRadians(getTurretLocal());

        if (turretAngle > ActuationConstants.Launcher.turretMaxAngle && value > 0) {
            turret.setPower(0);
            turret.setTargetPosition(turret.getCurrentPosition());
        } else if (turretAngle < -ActuationConstants.Launcher.turretMaxAngle && value < 0) {
            turret.setPower(0);
            turret.setTargetPosition(turret.getCurrentPosition());
        } else {
            turret.setPower(power);
            turret.setTargetPosition(turret.getCurrentPosition()+value);
        }
        telemetry.addData("Turret Angle", Math.toDegrees(turretAngle));
        telemetry.addData("Max Angle", ActuationConstants.Launcher.turretMaxAngle);
    }

    public static void setTurret(double angle) {
        while (Math.abs(AngleUnit.normalizeRadians(getTurretGlobal()) - AngleUnit.normalizeRadians(angle)) >= Math.toRadians(0.5)) {
            turretMoveTowards(angle);
        }
        turret.setPower(0);
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

