package org.firstinspires.ftc.teamcode.utility.cameraVision;

import static org.firstinspires.ftc.teamcode.utility.Actuation.getLLResult;
import static org.firstinspires.ftc.teamcode.utility.Actuation.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
@TeleOp(name = "Odometry limelight")
@Config
public class OdometryLimelight extends OpMode {

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        Actuation.setupLimelight(0);
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());

                telemetry.update();
            }
        }

    }
}
