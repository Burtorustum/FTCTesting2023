package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleeveDetector;
import org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline;
import org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage;

@TeleOp(name = "Vision Tuner", group = "Tuning")
public class VisionTuner extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    RGBSignalSleeveDetector detector = new RGBSignalSleeveDetector(this);
    dashboard.startCameraStream(detector.webcam, 0);

    waitForStart();
    while (opModeIsActive()) {
      // This telemetry specifically for viewing output of RGBSignalSleeveDetector/Pipeline on FTC Dash.

      telemetry.addData("Vision | Determination", detector.getDetermination());

      StringBuilder select = new StringBuilder();
      for (PipelineStage p : PipelineStage.values()) {
        if (RGBSignalSleevePipeline.stageSelect == p) {
          select.append("*").append(p.name()).append("*");
        } else {
          select.append(p.name());
        }
        select.append(" -> ");
      }
      select.delete(select.length() - 4, select.length());

      telemetry.addLine("Vision | Stage Selection");
      telemetry.addLine(select.toString());

      telemetry.update();
    }

    detector.stopStreaming();
  }
}