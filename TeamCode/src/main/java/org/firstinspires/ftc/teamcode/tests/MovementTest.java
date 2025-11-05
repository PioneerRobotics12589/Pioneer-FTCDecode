package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Movement Test")
@Config
public class MovementTest extends OpMode {
    private Pose pos1, pos2, pos3, pos4;
    private final ElapsedTime runtime = new ElapsedTime();
    private final DataLogger dl = new DataLogger("MovementTestData");
    private double t0, t1, t2;
    private double time, prevTime;
    public static long delay;
    public static double move, turn, strafe, moveInt, turnInt, strafeInt;

    public MovementTest() throws IOException {
    }

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        runtime.reset();
        pos1 = pos2 = pos3 = pos4 = OttoCore.robotPose;
        time = runtime.seconds();
        prevTime = time;
    }

    @Override
    public void loop() {
        updatePoses();

        move = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        Actuation.drive(move, turn, strafe);

        double currentTime = runtime.seconds();
        double dt = currentTime-prevTime;
        prevTime = currentTime;

        double vx1 = (pos2.x - pos1.x) / (t0) * 0.0254;
        double vx2 = (pos3.x-pos2.x)/(t1)*0.0254;
        double vx3 = (pos4.x-pos3.x)/(t2)*0.0254;

        double vy1 = (pos2.y-pos1.y)/(t0)*0.0254;
        double vy2 = (pos3.y-pos2.y)/(t1)*0.0254;
        double vy3 = (pos4.y-pos3.y)/(t2)*0.0254;

        double vx = (vx1 +vx2+vx3)/3.0;
        double vy = (vy1+vy2+vy3)/3.0;
        double v_tot = Math.sqrt(vx*vx + vy*vy);

        double vf = vx*Math.sin(pos4.heading) + vy*Math.cos(pos4.heading);
        double vs = vx*Math.cos(pos4.heading) + vy*Math.sin(pos4.heading);

        moveInt += move * dt;
        strafeInt += strafe * dt;
        turnInt += turn * dt;

        Actuation.packet.put("vf", vf);
        Actuation.packet.put("vs", vs);
        Actuation.packet.put("h", pos4.heading);
        Actuation.packet.put("move", move);
        Actuation.packet.put("strafe", strafe);
        Actuation.packet.put("turn", turn);
        Actuation.packet.put("turnInt", turnInt);
        Actuation.packet.addLine("move="+move+" turn="+turn+" strafe="+strafe);
        Actuation.updateTelemetry();

        try {
            dl.addDataLine("r"+currentTime+"c"+move+"c"+turn+"c"+strafe+"c"+turnInt+"c"+vx+"c"+vy+"c"+pos4.heading);
        } catch (IOException e) {
            throw new RuntimeException(e);

        }

    }

    @Override
    public void stop() {
        dl.close();
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
    }
}