package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="Starter Robot (Awe∑ Sauce)")
@Config
public class StarterRobot extends OpMode {
    CRServo left, right;
    DcMotor leftWheel, rightWheel;
    DcMotorEx flywheel;
    public static int artifactDelay = 200, launchDelay = 400, launchDeriv = 1000;
    public static boolean servos;

    public static double servoSpeed = 1.0;
    public static double fwkp = 0.29, fwki, fwkd;

    public static double longVel = 1700, shortVel = 1400, flywheelVel;

    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();

    public void init() {
        left = hardwareMap.crservo.get("leftLoader");
        right = hardwareMap.crservo.get("rightLoader");

        leftWheel = hardwareMap.dcMotor.get("leftDrive");
        rightWheel = hardwareMap.dcMotor.get("rightDrive");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(fwkp, fwki, fwkd, 0));

        dashboard = FtcDashboard.getInstance();
    }

    public void loop() {
        double move = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        leftWheel.setPower(move - turn);
        rightWheel.setPower(move + turn);

        if(gamepad1.right_bumper) {
            flywheelVel = longVel;
        } else if (gamepad1.left_bumper) {
            flywheelVel = shortVel;
        } else if (gamepad1.right_trigger > 0.5 || servos) {
            startFlywheel(flywheelVel);
            startServos();

            delay(artifactDelay);
            stopServos();

            delay(launchDelay);

            if (gamepad1.right_trigger < 0.5 || !servos) {
                stopFlywheel();
            }
        } else {
            stopFlywheel();
            stopServos();
        }

        packet.put("Flywheel Velocity", flywheel.getVelocity());
        packet.put("Flywheel Target Velocity", flywheelVel);
        dashboard.sendTelemetryPacket(packet);
    }

    public void startFlywheel(double target) {
        double prevVel = flywheel.getVelocity();

        while (flywheel.getVelocity() != target) {
            flywheel.setVelocity(target);

            packet.put("Flywheel Acceleration", prevVel-flywheel.getVelocity());
            dashboard.sendTelemetryPacket(packet);

            prevVel = flywheel.getVelocity();
        }
    }
    public void stopFlywheel() {
        flywheel.setVelocity(0);
    }
    public void startServos() {
        left.setPower(-servoSpeed);
        right.setPower(servoSpeed);
    }
    public void stopServos() {
        left.setPower(0);
        right.setPower(0);
    }
    public void delay(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException();
        }
    }
}