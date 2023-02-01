package org.firstinspires.ftc.teamcode.Robot.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class Vision {

  public final OpenCvWebcam webcam;
  protected final Telemetry telemetry;

  protected Vision(LinearOpMode opMode) {
    HardwareMap hwMap = opMode.hardwareMap;
    telemetry = opMode.telemetry;

    int cameraMonitorViewId = hwMap.appContext.getResources()
        .getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance()
        .createWebcam(hwMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
  }

  protected void setPipelineAndOpen(OpenCvPipeline pipeline) {
    webcam.setPipeline(pipeline);
    webcam.setMillisecondsPermissionTimeout(2500);

    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        webcam.startStreaming(480, 360, OpenCvCameraRotation.UPRIGHT); // 360p
        //webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); // 480p
        //webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); // 720p

      }

      @Override
      public void onError(int errorCode) {

      }
    });
  }

  public void stopStreaming() {
    webcam.pauseViewport();
    webcam.stopStreaming();
  }
}