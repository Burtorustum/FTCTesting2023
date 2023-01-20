package org.firstinspires.ftc.teamcode.Robot.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.text.DecimalFormat;

@Config
public class ConeDetector extends Vision {

    private PARK_LOCATION determination = PARK_LOCATION.UNKNOWN;

    // Dashboard-configurable tuning vars
    public static Rect cropRect = new Rect(250, 250, 300, 300);
    public static Size blurKSize = new Size(3, 3);
    public static int erodeIterations = 12;
    public static int dilateIterations = 2;

    private PipelineStage stageSelect = PipelineStage.OUTPUT;

    public enum PipelineStage {
        INPUT, CROP,
        BLUR, ERODE, DILATE,
        CVT_COLOR, OUTPUT;

        public Mat result;
        PipelineStage() {
            this.result = new Mat();
        }
    }

    public final OpenCvPipeline pipeline = new OpenCvPipeline() {
        @Override
        public Mat processFrame(Mat input) {
            // Input
            PipelineStage.INPUT.result = input;

            // Crop
            try {
                PipelineStage.CROP.result = input.submat(cropRect);
            } catch (Exception ignored) {
                PipelineStage.CROP.result = input;
            }

            // Blur / Erode / Dilate
            Imgproc.blur(PipelineStage.CROP.result, PipelineStage.BLUR.result, blurKSize, new Point(-1, -1));
            Imgproc.erode(PipelineStage.BLUR.result, PipelineStage.ERODE.result, new Mat(), new Point(-1, -1), erodeIterations, Core.BORDER_DEFAULT);
            Imgproc.dilate(PipelineStage.ERODE.result, PipelineStage.DILATE.result, new Mat(), new Point(-1, -1), dilateIterations, Core.BORDER_DEFAULT);

            // Convert Color
            Imgproc.cvtColor(PipelineStage.DILATE.result, PipelineStage.CVT_COLOR.result, Imgproc.COLOR_RGBA2RGB);

            // Output
            PipelineStage.OUTPUT.result = PipelineStage.CVT_COLOR.result;

            // Return the desired frame from the pipeline to the camera stream
            return stageSelect.result;
        }
    };

    public ConeDetector(LinearOpMode opMode) {
        super(opMode);
        setPipelineAndOpen(pipeline);
    }

    @Override
    public void update(Telemetry telemetry) {
        Mat output = PipelineStage.OUTPUT.result;
        if (output == null) {
            this.determination = PARK_LOCATION.UNKNOWN;
        } else {
            try {
                double r = 0;
                double g = 0;
                double b = 0;

                for (int x = 0; x < output.cols(); x++) {
                    for (int y = 0; y < output.rows(); y++) {
                        double[] values = output.get(y, x);
                        r += values[0];
                        g += values[1];
                        b += values[2];
                    }
                }

                double outputArea = output.cols() * output.rows();
                double rAvg = r / outputArea;
                double gAvg = g / outputArea;
                double bAvg = b / outputArea;
                double highestAvg = Math.max(rAvg, Math.max(gAvg, bAvg));

                if (highestAvg == bAvg) {
                    this.determination = PARK_LOCATION.ONE;
                } else if (highestAvg == rAvg) {
                    this.determination = PARK_LOCATION.TWO;
                } else {
                    this.determination = PARK_LOCATION.THREE;
                }
                DecimalFormat df = new DecimalFormat("#.###");
                telemetry.addData("Vision | Avg Red", df.format(rAvg));
                telemetry.addData("Vision | Avg Green", df.format(gAvg));
                telemetry.addData("Vision | Avg Blue", df.format(bAvg));
            } catch (Exception e) {
                telemetry.addData("Vision | Error", e.getMessage());
            }
        }

        telemetry.addData("Vision | Determination", this.determination);

        StringBuilder select = new StringBuilder();
        for (PipelineStage p : PipelineStage.values()) {
            if (stageSelect == p) {
                select.append("*").append(p.name()).append("*");
            } else {
                select.append(p.name());
            }
            select.append(" -> ");
        }
        select.delete(select.length() - 4, select.length());

        telemetry.addLine("Vision | Stage Selection");
        telemetry.addLine(select.toString());
    }

    public PARK_LOCATION getDetermination() {
        return this.determination;
    }

    public void setStageSelect(PipelineStage stage) {
        stageSelect = stage;
    }
}