package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double t0, t1, t2;
    private double time, prevTime;
    public static long delay;
    public static double move, turn, strafe, turnInt;
    private BufferedWriter writer;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        runtime.reset();
        pos1 = pos2 = pos3 = pos4 = OttoCore.robotPose;
        time = runtime.seconds();
        prevTime = time;
        try {
            File logFile = AppUtil.getInstance().getSettingsFile("MovementLog.txt");
            writer = new BufferedWriter(new FileWriter(logFile, false)); // false = overwrite
            writer.write("time move vf vs\n");
        } catch (IOException e) {
            telemetry.addLine("Error creating log file: " + e.getMessage());
        }
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

        if (writer != null) {
            try {
                writer.write(("r"+vf+"c"+vs+"c"+pos4.heading+"c"+move+"c"+strafe+"c"+turn+"c"+turnInt));
                writer.flush();
            } catch (IOException e) {
                telemetry.addLine("Error writing to log: " + e.getMessage());
            }
        }
    }

    @Override
    public void stop() {
        try {
            if (writer != null) {
                writer.flush();
                writer.close();
            }
        } catch (IOException e) {
            telemetry.addLine("Error closing log file: " + e.getMessage());
        }
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