package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Blocker Test", group = "tests")
public class BlockerTest extends OpMode {
    public static double angle;
    public static double intakePower;
    public static double transferPower;
    private Servo blocker;
    private DcMotor intake, transfer;

    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, "blocker");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
    }

    @Override
    public void loop() {
        blocker.setPosition(angle);
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }
}
