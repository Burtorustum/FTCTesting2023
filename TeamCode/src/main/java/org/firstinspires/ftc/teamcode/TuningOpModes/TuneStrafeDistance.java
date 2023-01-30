package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Robot.TestBot;

// ASSUMES DRIVE PID ALREADY TUNED
@Config
@TeleOp(name = "Tune Strafe Distance Mult", group = "Tuning")
public class TuneStrafeDistance extends LinearOpMode {

  public static int DISTANCE_TARGET = 24;
  public static DistanceUnit UNIT = DistanceUnit.INCH;

  @Override
  public void runOpMode() throws InterruptedException {

    // Init a robot object
    TestBot robot = new TestBot(this);

    // Wait for opmode start
    waitForStart();
    robot.waitForStart();

    while (opModeIsActive() && !gamepad1.a) {
      telemetry.addLine("To start, press a!");
      telemetry.update();
    }
    telemetry.clear();

    while (opModeIsActive()) {
      robot.drivetrain.strafeDistance(DISTANCE_TARGET, UNIT);
      robot.runUntil(robot.drivetrain::driveComplete);

      while (opModeIsActive() && !gamepad1.a) {
        telemetry.addData("Target Distance:", DISTANCE_TARGET + " " + UNIT.name());
        telemetry.addData("Multiplier:", MecanumDrivetrain.STRAFE_MULTIPLIER);
        telemetry.addLine("Measure the distance the bot moved, and update MULTIPLIER accordingly");
        telemetry.addLine("To run again, press a!");
        telemetry.update();
      }
      telemetry.clearAll();

      robot.drivetrain.setMode(RunMode.STOP_AND_RESET_ENCODER);

      while (opModeIsActive() && !robot.drivetrain.isEncoderResetFinished()) {
        telemetry.addLine("Resetting encoders");
        telemetry.update();
      }
      robot.drivetrain.setMode(RunMode.RUN_WITHOUT_ENCODER);
      telemetry.clearAll();
    }

  }
}