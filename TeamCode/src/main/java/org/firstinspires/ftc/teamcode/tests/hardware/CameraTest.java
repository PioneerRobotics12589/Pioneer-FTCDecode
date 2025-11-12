package org.firstinspires.ftc.teamcode.tests.hardware;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
                 double x = fiducial.getTargetXDegrees();
                 double y = fiducial.getTargetYDegrees();
                 double strafeDistance = fiducial.getRobotPoseTargetSpace().getPosition().y;
                 telemetry.addData("Fiducial: " + id, "is " + strafeDistance + " meters away");
             }
         }

         telemetry.update();
     }
}
