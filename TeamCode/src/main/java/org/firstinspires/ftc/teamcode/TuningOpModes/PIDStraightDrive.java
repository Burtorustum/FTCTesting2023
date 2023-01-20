package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.TestBot;

public class PIDStraightDrive extends LinearOpMode {
    public static int targetIn = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        TestBot robot = new TestBot(this);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            robot.drivetrain.driveDistance(targetIn, DistanceUnit.CM);
            robot.update();
        }
    }
}
