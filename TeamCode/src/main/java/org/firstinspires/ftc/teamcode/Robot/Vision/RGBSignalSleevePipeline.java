package org.firstinspires.ftc.teamcode.Robot.Vision;

import static org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage.BLUR;
import static org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage.CROP;
import static org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage.CVT_COLOR;
import static org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage.DILATE;
import static org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage.ERODE;
import static org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage.INPUT;
import static org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleevePipeline.PipelineStage.OUTPUT;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RGBSignalSleevePipeline extends OpenCvPipeline {

  public static Rect cropRect = new Rect(new Point(1, 1), new Point(300, 300));
  public static Size blurKSize = new Size(3, 3);
  public static int erodeIterations = 12;
  public static int dilateIterations = 2;

  public static PipelineStage stageSelect = PipelineStage.OUTPUT;

  public PARK_LOCATION determination = PARK_LOCATION.UNKNOWN;

  @Override
  public Mat processFrame(Mat input) {
    // Input
    INPUT.result = input;

    // Crop
    try {
      CROP.result = input.submat(cropRect);
    } catch (Exception ignored) {
      CROP.result = input;
    }

    // Blur / Erode / Dilate (overkill for this application)
    Imgproc.blur(CROP.result, BLUR.result, blurKSize,
        new Point(-1, -1));
    Imgproc.erode(BLUR.result, ERODE.result, new Mat(),
        new Point(-1, -1), erodeIterations, Core.BORDER_DEFAULT);
    Imgproc.dilate(ERODE.result, DILATE.result, new Mat(),
        new Point(-1, -1), dilateIterations, Core.BORDER_DEFAULT);

    // Convert Color
    Imgproc.cvtColor(DILATE.result, CVT_COLOR.result, Imgproc.COLOR_RGBA2RGB);

    // Output
    OUTPUT.result = CVT_COLOR.result;

    updateDetermination(OUTPUT.result);

    // Return the desired frame from the pipeline to the camera stream
    return stageSelect.result;
  }

  public void updateDetermination(Mat image) {

    if (image == null || image.empty()) {
      this.determination = PARK_LOCATION.UNKNOWN;
      return;
    }

    double r = 0;
    double g = 0;
    double b = 0;

    for (int row = 0; row < image.rows(); row++) {
      for (int col = 0; col < image.cols(); col++) {
        double[] values = image.get(row, col);
        r += values[0];
        g += values[1];
        b += values[2];
      }
    }

    double maxColorVal = Math.max(r, Math.max(g, b));

    if (maxColorVal == b) {
      this.determination = PARK_LOCATION.ONE;
    } else if (maxColorVal == r) {
      this.determination = PARK_LOCATION.TWO;
    } else {
      this.determination = PARK_LOCATION.THREE;
    }
  }

  public enum PipelineStage {
    INPUT, CROP,
    BLUR, ERODE, DILATE,
    CVT_COLOR, OUTPUT;

    public Mat result;

    PipelineStage() {
      this.result = new Mat();
    }
  }
}