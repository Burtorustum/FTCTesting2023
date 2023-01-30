package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Vision.ConeDetector;
import org.firstinspires.ftc.teamcode.Robot.Vision.Vision;

@Config
@TeleOp(name = "Vision Tuner", group = "Tuning")
public class VisionTuning extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    // Replace with different detector as-needed
    Vision detector = new ConeDetector(this);
    dashboard.startCameraStream(detector.webcam, 0);

    waitForStart();
    while (opModeIsActive() && !isStopRequested()) {
      detector.update(telemetry);
      telemetry.update();
    }

    detector.stopStreaming();
  }
}