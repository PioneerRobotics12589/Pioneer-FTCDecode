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
    public static double angle1, angle2;
    public static double intakePower;
    public static double transferPower;
    private Servo blocker1, blocker2;
    private DcMotor intake, transfer;

    @Override
    public void init() {
        blocker1 = hardwareMap.get(Servo.class, "blocker1");
        blocker2 = hardwareMap.get(Servo.class, "blocker2");
        /*intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");*/
    }

    @Override
    public void loop() {
        blocker1.setPosition(angle1);
        blocker2.setPosition(angle2);
        //intake.setPower(intakePower);
        //transfer.setPower(transferPower);
    }
}
