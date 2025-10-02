package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(name="Wheel Test", group="tests")
public class WheelTest extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        if (hardwareMap.dcMotor.contains("frontLeft")) {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (hardwareMap.dcMotor.contains("frontRight")) {
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (hardwareMap.dcMotor.contains("backLeft")) {
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (hardwareMap.dcMotor.contains("backRight")) {
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    @Override
    public void loop() {
        if (gamepad1.square) frontLeft.setPower(0.5);
        else frontLeft.setPower(0.0);

        if (gamepad1.triangle) frontRight.setPower(0.5);
        else frontRight.setPower(0.0);

        if (gamepad1.cross) backLeft.setPower(0.5);
        else backLeft.setPower(0.0);

        if (gamepad1.circle) backRight.setPower(0.5);
        else backRight.setPower(0.0);
    }
}