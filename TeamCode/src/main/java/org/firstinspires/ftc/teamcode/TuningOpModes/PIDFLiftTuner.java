package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Lift.Height;
import org.firstinspires.ftc.teamcode.Robot.TestBot;

@TeleOp(name = "PIDF Lift Tuner", group = "Tuning")
public class PIDFLiftTuner extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    TestBot robot = new TestBot(this);

    telemetry.addLine("Ready!");
    telemetry.update();

    robot.waitForStart();
    robot.lift.toManualLift();

    robot.runUntil(() -> {
      telemetry.addLine("First, tune kg until the lift doesn't move on its own at any height");
      telemetry.addLine("Once tuned, press y!");

      robot.lift.manualLiftPower(gamepad1.right_stick_y);

      return gamepad1.y;
    });
    telemetry.clearAll();
    robot.lift.toAutomaticLift();

    robot.runWhile(() -> {
      telemetry.addLine("Now tune PID until movement to desired height is clean and error is low.");
      telemetry.addLine();
      telemetry.addData("Error", robot.lift.getError());

      if (gamepad1.a) {
        robot.lift.setHeight(Height.INTAKE);
      } else if (gamepad1.dpad_up) {
        robot.lift.setHeight(Height.HIGH);
      } else if (gamepad1.dpad_right) {
        robot.lift.setHeight(Height.MEDIUM);
      } else if (gamepad1.dpad_down) {
        robot.lift.setHeight(Height.LOW);
      } else if (gamepad1.dpad_left) {
        robot.lift.setHeight(Height.GROUND);
      }

      return true;
    });
  }
}