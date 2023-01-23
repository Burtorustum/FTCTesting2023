package org.firstinspires.ftc.teamcode.ExampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.TestBot;

@Autonomous(name = "Example Auton", group = "Example")
public class ExampleLinearAuton extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {

    // Init a robot object
    TestBot robot = new TestBot(this);

    // Wait for opmode start
    robot.runUntil(this::opModeIsActive);

    // Turn to 135d
    robot.drivetrain.turn(135);
    robot.runUntil(robot.drivetrain::driveComplete);

    // Intake until cone picked up
    //robot.intake.intake();
    //robot.runUntil(robot.intake::coneDetected);

    // Turn to -90d (270d)
    //robot.intake.idle();
    robot.drivetrain.turn(-90);
    robot.runUntil(robot.drivetrain::driveComplete);

    // Drive 20cm forward
    robot.drivetrain.driveDistance(20, DistanceUnit.CM);
    robot.runUntil(robot.drivetrain::driveComplete);

    // Do nothing for 3 seconds
    robot.runForTime(3000);

    // Turn to 0d
    robot.drivetrain.turn(0);
    robot.runUntil(robot.drivetrain::driveComplete);

    // Wait for stop
    robot.runUntilStop();
  }
}
