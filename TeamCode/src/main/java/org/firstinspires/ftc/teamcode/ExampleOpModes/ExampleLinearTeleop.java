package org.firstinspires.ftc.teamcode.ExampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Lift.ClawPosition;
import org.firstinspires.ftc.teamcode.Robot.Lift.Height;
import org.firstinspires.ftc.teamcode.Robot.Lift.SwivelPosition;
import org.firstinspires.ftc.teamcode.Robot.TestBot;

@TeleOp(name = "Example Teleop", group = "Example")
public class ExampleLinearTeleop extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {

    // Init a robot object
    TestBot robot = new TestBot(this);

    // Wait for opmode start
    waitForStart();

    robot.start();
    while (opModeIsActive() && !isStopRequested()) {
      double y = -gamepad1.left_stick_y; // Remember, this is reversed!
      double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
      double rx = gamepad1.right_stick_x;
      robot.drivetrain.fieldCentric(y, x, rx);

      // Control Lift height and swivel
      if (gamepad2.guide) {
        robot.lift.setHeight(Height.INTAKE);
        robot.lift.setSwivelPosition(SwivelPosition.IN);
      } else if (gamepad2.dpad_up) {
        robot.lift.setHeight(Height.HIGH);
        //robot.lift.setSwivelPosition(SwivelPosition.OUT);
      } else if (gamepad2.dpad_right) {
        robot.lift.setHeight(Height.MEDIUM);
        //robot.lift.setSwivelPosition(SwivelPosition.OUT);
      } else if (gamepad2.dpad_down) {
        robot.lift.setHeight(Height.LOW);
        robot.lift.setSwivelPosition(SwivelPosition.IN);
      } else if (gamepad2.dpad_left) {
        robot.lift.setHeight(Height.GROUND);
        robot.lift.setSwivelPosition(SwivelPosition.IN);
      }

      // Control lift shift
      if (gamepad2.right_bumper) {
        robot.lift.shiftDown();
      } else {
        robot.lift.shiftUp();
      }

      // Control lift claw
      if (gamepad2.right_trigger > 0.1) {
        robot.lift.setClawPosition(ClawPosition.CLOSED);
      } else if (gamepad2.left_trigger > 0.1) {
        robot.lift.setClawPosition(ClawPosition.OPEN);
      }

      if (gamepad2.a) {
        robot.lift.setSwivelPosition(SwivelPosition.IN);
      } else if (gamepad2.b) {
        robot.lift.setSwivelPosition(SwivelPosition.RIGHT);
      } else if (gamepad2.y) {
        robot.lift.setSwivelPosition(SwivelPosition.OUT);
      }

      if (gamepad2.left_stick_button) {
        robot.lift.toManualLift();
      }

      // Manual control
      if (!robot.lift.autoLift) {
        robot.lift.manualLiftPower(-gamepad2.right_stick_y);
      }

      robot.update();
    }
  }
}