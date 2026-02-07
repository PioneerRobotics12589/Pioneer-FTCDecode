package org.firstinspires.ftc.teamcode.utility.cameraVision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utility.imu.IMUControl;

@TeleOp(name="AprilTagTest")


public class AprilTagTest extends OpMode {
    private Limelight3A limelight;

    private YawPitchRollAngles botpose() {
        return null;
    }

    private LLResult result;
    Pose3D botPose= result.getBotpose_MT2();




    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);     //pipeline #
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

    }

    @Override
    public void start() {
        limelight.start();
    }
    @Override
   public void loop() {

            YawPitchRollAngles orientation = botpose();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                IMUControl.setup(hardwareMap,
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
                botPose = llResult.getBotpose_MT2(); // problem line: llResult on a null object
                telemetry.addData("Tx", llResult.getTx());  // store data and display on driver station
                telemetry.addData("Ty", llResult.getTy()); // store data and display on driver station
                telemetry.addData("Ta", llResult.getTa()); // store data and display on driver station
            }


            telemetry.update(

            );
        }




}


