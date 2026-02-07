package org.firstinspires.ftc.teamcode.utility.cameraVision;


import static org.firstinspires.ftc.teamcode.utility.Actuation.backLeft;
import static org.firstinspires.ftc.teamcode.utility.Actuation.getLLResult;
import static org.firstinspires.ftc.teamcode.utility.Actuation.limelight;
import static org.firstinspires.ftc.teamcode.utility.Actuation.turretMoveTowards;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
@TeleOp(name="AprilTagTracking")
public class AprilTagTracking extends OpMode {
    private Limelight3A limelight;
    private static final float KpAim = -0.1f;
    private static final float KpDistance = -0.1f;
    private static final float minAimCommand = 0.05f;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
//        Actuation.setupLimelight(0);
    }

    @Override
    public void loop() {

        LLResult res = limelight.getLatestResult();
        if (!res.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fid = res.getFiducialResults().get(0);

            // commands[0] = left_command, commands[1] = right_command
            double tx = Math.toRadians(fid.getTargetXDegrees());
            double ty = Math.toRadians(fid.getTargetYDegrees());


            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", ty);

            double headingError = -tx;
            double distanceError = -ty;
            double steeringAdjust = 0.0f;

if (gamepad1.crossWasPressed()){
    Actuation.turretMoveTowards();
}
            if (gamepad1.crossWasReleased()){
                    Actuation.turretMoveTowards((Actuation.getTurretLocal()-tx));
            }
            

            if (tx > 1.0f) {
                steeringAdjust = KpAim * headingError - minAimCommand;
            } else if (tx < -1.0f) {
                steeringAdjust = KpAim * headingError + minAimCommand;
            }
            Actuation.turretMoveTowards(Actuation.getTurretLocal()-tx);



            double distanceAdjust = KpDistance * distanceError;
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("turnRate", steeringAdjust * distanceAdjust);

        } else {
            telemetry.addData("turnRate", 0);
        }
        telemetry.update();
    }
}
