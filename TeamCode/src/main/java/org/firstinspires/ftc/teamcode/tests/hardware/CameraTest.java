package org.firstinspires.ftc.teamcode.tests.hardware;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.cameraVision.AprilTagDetection;

import java.util.List;

@TeleOp(name = "CameraTest", group = "tests")
public class CameraTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();
    }

    @Override
    public void loop(){
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fids) {
                int id = fiducial.getFiducialId();
                double[] position = AprilTagDetection.getGlobalPosition(fiducial);
                telemetry.addData("X relative to field center", position[0]);
                telemetry.addData("Y relative to field center", position[1]);
            }
        }

        telemetry.update();
    }
}