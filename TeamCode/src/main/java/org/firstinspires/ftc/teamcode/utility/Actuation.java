package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;

public class Actuation {
    public static boolean slowMode = false;
    private static boolean slowModeToggle = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotorEx flywheel;

    public static CRServo leftLoader, rightLoader;

    public static Telemetry telemetry;

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

        double multip = (slowMode) ? 0.5 : 1.0;

        frontLeft.setPower((move+strafe+turn) * multip);
        backLeft.setPower((move-strafe+turn) * multip);
        frontRight.setPower((move-strafe-turn) * multip);
        backRight.setPower((move+strafe-turn) * multip);

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
        }
        else {
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

    public static void updateTelemetry() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}