package org.firstinspires.ftc.teamcode.Robot.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ConeDetector extends Vision {
    public enum PipelineStage {
        INPUT, ERODE, DILATE, CVT_COLOR, CROP
    }

    private final Mat erodeOut = new Mat();
    private final Mat dilateOut = new Mat();

    private final Mat cvtColorOut = new Mat();

    public static Rect cropRect = new Rect(280, 140, 50, 50);
    public static int erodeIterations = 3;
    public static int dilateIterations = 5;
    private Mat cropOut;

    private PipelineStage stageSelect = PipelineStage.CROP;

    public final OpenCvPipeline pipeline = new OpenCvPipeline() {
        @Override
        public Mat processFrame(Mat input) {
            // Erode / Dilate -----------------------------------
            Imgproc.erode(input, erodeOut, new Mat(), new Point(-1, -1), erodeIterations, Core.BORDER_CONSTANT);
            Imgproc.dilate(erodeOut, dilateOut, new Mat(), new Point(-1, -1), dilateIterations, Core.BORDER_CONSTANT);

            // Convert Color -------------------------------
            Imgproc.cvtColor(dilateOut, cvtColorOut, Imgproc.COLOR_RGBA2RGB);

            // Crop --------------------------------
            cropOut = new Mat(cvtColorOut, cropRect);

            switch (stageSelect) {
                case INPUT:
                    return input;
                case ERODE:
                    return erodeOut;
                case DILATE:
                    return dilateOut;
                case CVT_COLOR:
                    return cvtColorOut;
                default:
                    return cropOut;
            }
        }
    };

    private PARK_LOCATION determination = PARK_LOCATION.UNKNOWN;

    public ConeDetector(LinearOpMode opMode) {
        super(opMode);
        setPipelineAndOpen(pipeline);
    }

    @Override
    public void update(Telemetry telemetry) {
        if (cropOut == null) {
            this.determination = PARK_LOCATION.UNKNOWN;
        } else {
            try {
                double r = 0;
                double g = 0;
                double b = 0;

                for (int x = 0; x < cropRect.width; x++) {
                    for (int y = 0; y < cropRect.height; y++) {
                        double[] values = cropOut.get(x, y);
                        r += values[0];
                        g += values[1];
                        b += values[2];
                    }
                }

                double r_avg = r / cropRect.width * cropRect.height;
                double g_avg = g / cropRect.width * cropRect.height;
                double b_avg = b / cropRect.width * cropRect.height;

                double magRed = Math.sqrt(Math.pow((255 - r_avg), 2) + Math.pow(g_avg, 2) + Math.pow(b_avg, 2));
                double magGreen = Math.sqrt(Math.pow((r_avg), 2) + Math.pow(255 - g_avg, 2) + Math.pow(b_avg, 2));
                double magBlue = Math.sqrt(Math.pow((r_avg), 2) + Math.pow(g_avg, 2) + Math.pow(255 - b_avg, 2));

                double lowest = Math.min(magRed, Math.min(magGreen, magBlue));

                if (lowest == magBlue) {
                    this.determination = PARK_LOCATION.ONE;
                } else if (lowest == magRed) {
                    this.determination = PARK_LOCATION.TWO;
                } else {
                    this.determination = PARK_LOCATION.THREE;
                }
            } catch (Exception e) {
                telemetry.addData("Vision | Error", e.getMessage());
                telemetry.update();
            }
        }

        telemetry.addData("Vision | Parking Determination", this.determination);
        telemetry.addData("Vision | Stage Select", this.stageSelect.name());
    }

    public PARK_LOCATION getDetermination() {
        return this.determination;
    }

    public void setStageSelect(PipelineStage stage) {
        this.stageSelect = stage;
    }
}