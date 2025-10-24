package org.firstinspires.ftc.teamcode.tests.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="Starter Auto")
public class StarterAuto extends LinearOpMode {
    DcMotor left, right;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "leftDrive");
        right = hardwareMap.get(DcMotor.class, "rightDrive");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        sleep(1000);
        left.setPower(-1);
        right.setPower(-1);
        sleep(5000);
        left.setPower(1);
        right.setPower(-1);
        sleep(5000);
    }
}