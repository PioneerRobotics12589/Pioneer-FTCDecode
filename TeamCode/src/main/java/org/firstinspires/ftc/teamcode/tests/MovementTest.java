package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name="Movement Test")
@Config
public class MovementTest extends OpMode {
    private Pose pos1, pos2, pos3, pos4;
    private double vx1, vx2, vx3, vy1, vy2, vy3;
    private double ax1, ax2, ay1, ay2;
    private ElapsedTime runtime = new ElapsedTime();
    private double t0, t1, t2;
    private double time;
    public static long delay;
    public static double move, turn, strafe;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        runtime.reset();
        pos1 = OttoCore.robotPose;
        pos2 = OttoCore.robotPose;
        pos3 = OttoCore.robotPose;
        pos4 = OttoCore.robotPose;
        time = runtime.seconds();
    }

    @Override
    public void loop() {
        updatePoses();

        move = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        Actuation.drive(move, turn, strafe);

        vx1 = (pos2.x-pos1.x)/(t0)*0.0254;
        vx2 = (pos3.x-pos2.x)/(t1)*0.0254;
        vx3 = (pos4.x-pos3.x)/(t2)*0.0254;
        ax1 = (vx2-vx1)/((t0+t1)/2.0);
        ax2 = (vx3-vx2)/((t1+t2)/2.0);

        vy1 = (pos2.y-pos1.y)/(t0)*0.0254;
        vy2 = (pos3.y-pos2.y)/(t1)*0.0254;
        vy3 = (pos4.y-pos3.y)/(t2)*0.0254;
        ax1 = (vy2-vy1)/((t0+t1)/2.0);
        ax2 = (vy3-vy2)/((t1+t2)/2.0);

        Actuation.packet.put("ax", (ax1+ax2)/2.0);
        Actuation.packet.put("ay", (ay1+ay2)/2.0);
        Actuation.packet.put("vx", (vx1+vx2+vx3)/3.0);
        Actuation.packet.put("vy", (vy1+vy2+vy3)/3.0);
        Actuation.updateTelemetry();
    }

    public void updatePoses() {
        OttoCore.updatePosition();
        pos1 = new Pose(OttoCore.robotPose);
        time = runtime.seconds();

        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        OttoCore.updatePosition();
        pos2 = new Pose(OttoCore.robotPose);
        t0 = (runtime.seconds()-time);
        time = runtime.seconds();

        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        OttoCore.updatePosition();
        pos3 = new Pose(OttoCore.robotPose);
        t1 = (runtime.seconds()-time);
        time = runtime.seconds();

        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        OttoCore.updatePosition();
        pos4 = new Pose(OttoCore.robotPose);
        t2 = (runtime.seconds()-time);

        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}