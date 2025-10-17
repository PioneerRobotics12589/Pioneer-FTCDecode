package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@Autonomous(name="Starter Auto", group="tests")
public class StarterRobotAuto extends LinearOpMode {
    DcMotor leftWheel, rightWheel;

    public static double WheelSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Actuation.setup(hardwareMap, telemetry);
        leftWheel = hardwareMap.dcMotor.get("leftDrive");
        rightWheel = hardwareMap.dcMotor.get("rightDrive");
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        rightWheel.setPower(WheelSpeed);
        leftWheel.setPower(WheelSpeed);


    }
}
