package org.firstinspires.ftc.teamcode.ExampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TestBot;
import org.firstinspires.ftc.teamcode.Robot.Vision.ConeDetector;

@TeleOp(name = "Vision Tuner", group = "Tuning")
public class VisionTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TestBot robot = new TestBot(this);

        robot.runUntil(this::opModeIsActive);

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                robot.coneDetector.setStageSelect(ConeDetector.PipelineStage.CROP);
            } else if (gamepad1.b) {
                robot.coneDetector.setStageSelect(ConeDetector.PipelineStage.DILATE);
            } else if (gamepad1.y) {
                robot.coneDetector.setStageSelect(ConeDetector.PipelineStage.ERODE);
            }

            robot.update();
        }

        robot.coneDetector.closeCamera();
    }
}
