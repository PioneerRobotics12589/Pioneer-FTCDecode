package org.firstinspires.ftc.teamcode.tests.hardware;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.cameraVision.AprilTagDetection;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.List;

@TeleOp(name = "CameraTest", group = "tests")
public class CameraTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        OttoCore.robotPose = new Pose(0.0,0.0,0.0);
        limelight.start();
    }

    @Override
    public void loop(){
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
            Pose position = AprilTagDetection.getGlobalPos(fids);
            telemetry.addData("X relative to field center", position.x);
            telemetry.addData("Y relative to field center", position.y);
            telemetry.addData("Heading relative to field center", position.heading);
        }

        telemetry.update();
    }
}