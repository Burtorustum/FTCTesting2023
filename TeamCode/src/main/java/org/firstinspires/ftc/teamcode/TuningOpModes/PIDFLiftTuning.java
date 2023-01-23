package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.TestBot;

@Config
@TeleOp(name = "PIDF Lift Tuner", group = "Tuning")
public class PIDFLiftTuning extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    TestBot robot = new TestBot(this);

    telemetry.addLine("Ready!");
    telemetry.update();

    waitForStart();

    while (!isStopRequested()) {
      robot.lift.setVelocityTarget(gamepad1.right_stick_y);
      robot.update();
    }
  }
}