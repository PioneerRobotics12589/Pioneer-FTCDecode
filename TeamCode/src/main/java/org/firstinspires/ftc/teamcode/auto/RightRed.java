package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.OttoCore;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="Right Red", group="red")
public class RightRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);

        Trajectory preloads = new Trajectory(new Pose(52, -46, Math.toRadians(-48)));

        Trajectory launch = new Trajectory()
                .action(() -> Actuation.setFlywheel(ActuationConstants.Launcher.shortLaunch))
                .lineTo(new Pose(11, -14, Math.toRadians(-38)))
                .action(() -> sleep(500))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.runTransfer(true))
                .action(() -> sleep(2000))
                .action(() -> Actuation.intake.setPower(0.0))
                .action(() -> Actuation.runTransfer(false))
                .action(() -> Actuation.setFlywheel(0));

        Trajectory spike1 = new Trajectory()
                .lineTo(new Pose(13, -30, Math.toRadians(-82)))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed * 0.5))
                .action(() -> sleep(1000))
                .lineTo(new Pose(13, -50, Math.toRadians(-82)), 0.4, 0.5)
                .action(() -> Actuation.runTransfer(false))
                .action(() -> Actuation.intake.setPower(0.0));

        Trajectory spike2 = new Trajectory()
                .lineTo(new Pose(-11, -30, Math.toRadians(-82)))
                .action(() -> Actuation.intake.setPower(ActuationConstants.Intake.intakeSpeed))
                .action(() -> Actuation.transfer.setPower(ActuationConstants.Intake.transferSpeed * 0.5))
                .action(() -> sleep(1000))
                .lineTo(new Pose(-11, -55, Math.toRadians(-82)), 0.4, 0.5)
                .action(() -> Actuation.runTransfer(false))
                .action(() -> Actuation.intake.setPower(0.0));

        waitForStart();

        preloads.run();
        launch.run();
        spike1.run();
        launch.run();
        spike2.run();
        launch.run();
    }
}
