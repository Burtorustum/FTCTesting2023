package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Vision.ConeDetector;

@Config
@TeleOp(name = "Vision Tuner", group = "Tuning")
public class VisionTuning extends LinearOpMode {
    public static ConeDetector.PipelineStage stage = ConeDetector.PipelineStage.OUTPUT;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ConeDetector detector = new ConeDetector(this);
        dashboard.startCameraStream(detector.webcam, 0);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            detector.setStageSelect(stage);
            detector.update(telemetry);
            telemetry.update();
        }

        detector.closeCamera();
    }
}