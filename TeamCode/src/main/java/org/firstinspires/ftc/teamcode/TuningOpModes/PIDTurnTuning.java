package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TestBot;

// See https://www.ctrlaltftc.com/the-pid-controller for more info on PID control
// Designed for use with FTC Dashboard

@Config
@TeleOp(name = "PID Turn Tuner", group = "Tuning")
public class PIDTurnTuning extends LinearOpMode {
    public static int target = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        TestBot robot = new TestBot(this);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            robot.drivetrain.turn(target);
            robot.update();
        }
    }
}