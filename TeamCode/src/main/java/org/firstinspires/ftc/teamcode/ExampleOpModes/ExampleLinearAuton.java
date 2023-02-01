package org.firstinspires.ftc.teamcode.ExampleOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.TestBot;
import org.firstinspires.ftc.teamcode.Robot.Vision.RGBSignalSleeveDetector;

@Config
@Autonomous(name = "Example Auton", group = "Example")
public class ExampleLinearAuton extends LinearOpMode {

  public static int FORWARD_DIST = 24;

  public static int PARK_ONE_DIST = -24;
  public static int PARK_TWO_DIST = 0;
  public static int PARK_THREE_DIST = 24;

  private int parkDistance;

  @Override
  public void runOpMode() throws InterruptedException {

    // Init a robot object
    TestBot robot = new TestBot(this);

    // Enable vision for this opmode
    RGBSignalSleeveDetector detector = new RGBSignalSleeveDetector(this);

    // Wait for opmode start
    robot.waitForStart();

    // disable vision once auton has begun
    detector.stopStreaming();

    // Switch strafe distance based on vision determination
    switch (detector.getDetermination()) {
      case UNKNOWN:
      case ONE:
        parkDistance = PARK_ONE_DIST;
        break;
      case TWO:
        parkDistance = PARK_TWO_DIST;
        break;
      case THREE:
        parkDistance = PARK_THREE_DIST;
        break;
    }

    // Drive forward
    robot.drivetrain.driveDistance(FORWARD_DIST, DistanceUnit.INCH);
    robot.runUntil(robot.drivetrain::driveComplete);

    // Strafe to park
    robot.drivetrain.strafeDistance(parkDistance, DistanceUnit.INCH);
    robot.runUntil(robot.drivetrain::driveComplete);

    // Wait for stop
    robot.runUntilStop();
  }
}