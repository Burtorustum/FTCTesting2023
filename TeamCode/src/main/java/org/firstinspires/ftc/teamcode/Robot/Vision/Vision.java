package org.firstinspires.ftc.teamcode.Robot.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Structure.Subsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class Vision extends Subsystem {

  public final OpenCvWebcam webcam;

  protected Vision(LinearOpMode opMode) {
    super(opMode);

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
        webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); //UPSIDE_DOWN or UPRIGHT
      }

      @Override
      public void onError(int errorCode) {

      }
    });
  }

  public void closeCamera() {
    webcam.closeCameraDeviceAsync(() -> {
    });
  }
}
