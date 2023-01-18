package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.TestBot;
import org.firstinspires.ftc.teamcode.Robot.Vision.ConeDetector;

@TeleOp(name = "Vision Tuner", group = "Tuning")
public class VisionTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        TestBot robot = new TestBot(this);

        dashboard.startCameraStream(robot.coneDetector.webcam, 0);

        robot.runUntil(this::opModeIsActive);

        ConeDetector.PipelineStage[] stages = ConeDetector.PipelineStage.values();
        int i = stages.length - 1;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_right && timer.seconds() > 0.12) {
                i = i + 1;
                timer.reset();
            } else if (gamepad1.dpad_left && timer.seconds() > 0.12) {
                i = i - 1;
                timer.reset();
            }

            if (i >= stages.length) {
                i = 0;
            } else if (i < 0) {
                i = stages.length - 1;
            }

            robot.coneDetector.setStageSelect(stages[i]);

            robot.update();
        }

        robot.coneDetector.closeCamera();
    }
}
