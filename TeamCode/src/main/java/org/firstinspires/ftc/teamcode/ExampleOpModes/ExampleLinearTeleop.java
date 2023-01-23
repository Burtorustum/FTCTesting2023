package org.firstinspires.ftc.teamcode.ExampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.TestBot;

@TeleOp(name = "Example Teleop", group = "Example")
public class ExampleLinearTeleop extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {

    // Init a robot object
    TestBot robot = new TestBot(this);

    // Wait for opmode start
    robot.runUntil(this::opModeIsActive);

    while (opModeIsActive() && !isStopRequested()) {
      double y = -gamepad1.left_stick_y; // Remember, this is reversed!
      double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
      double rx = gamepad1.right_stick_x;
      robot.drivetrain.fieldCentric(y, x, rx);
      robot.update();
    }
  }
}
