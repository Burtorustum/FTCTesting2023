package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "Test")
public class TestRun extends LinearOpMode {

  public enum PARK {
    ONE(-24), TWO(0), THREE(24);

    public final int inches;

    PARK(int inches) {
      this.inches = inches;
    }
  }

  public static double startX = -36;
  public static double startY = -63;
  public static double startHeading = 90;

  public static double gatherX = -55;
  public static double gatherY = -12;
  public static double gatherHeading = 180;

  public static double placeX = -32;
  public static double placeY = -8;
  public static double placeHeading = 45;

  public static double centralParkX = -36;
  public static double centralParkY = -12;
  public static double centralParkHeading = 90;

  public static PARK determination = PARK.ONE;

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    waitForStart();
    if (isStopRequested()) {
      return;
    }

    Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
    Pose2d gatherPose = new Pose2d(gatherX, gatherY, Math.toRadians(gatherHeading));
    Pose2d placePose = new Pose2d(placeX, placeY, Math.toRadians(placeHeading));
    Pose2d centralParkPose =
        new Pose2d(centralParkX, centralParkY, Math.toRadians(centralParkHeading));

    drive.setPoseEstimate(startPose);

    TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
        // Go to place Preload
        .forward(Math.abs(placeY - startY - 12))
        .UNSTABLE_addTemporalMarkerOffset(-2, () -> { /* Raise the lift */ })
        .splineTo(placePose.vec(), placePose.getHeading())
        .addDisplacementMarker(() -> { /* Release Cone  */ })
        .waitSeconds(0.5)
        .addDisplacementMarker(() -> { /* lower lift */})

        // Cycle 1
        .setReversed(true)
        .splineTo(gatherPose.vec(), gatherPose.getHeading())
        .UNSTABLE_addDisplacementMarkerOffset(-1, () -> { /* extend intake */ })
        .addDisplacementMarker(() -> { /* grab cone */ })
        .waitSeconds(1)
        .setReversed(false)
        .splineTo(placePose.vec(), placePose.getHeading())
        .addDisplacementMarker(() -> { /* Release Cone  */ })
        .waitSeconds(0.5)
        .addDisplacementMarker(() -> { /* lower lift */})

        // Cycle 2
        .setReversed(true)
        .splineTo(gatherPose.vec(), gatherPose.getHeading())
        .UNSTABLE_addDisplacementMarkerOffset(-1, () -> { /* extend intake */ })
        .addDisplacementMarker(() -> { /* grab cone */ })
        .waitSeconds(1)
        .setReversed(false)
        .splineTo(placePose.vec(), placePose.getHeading())
        .addDisplacementMarker(() -> { /* Release Cone  */ })
        .waitSeconds(0.5)
        .addDisplacementMarker(() -> { /* lower lift */})

        // Cycle 3
        .setReversed(true)
        .splineTo(gatherPose.vec(), gatherPose.getHeading())
        .UNSTABLE_addDisplacementMarkerOffset(-1, () -> { /* extend intake */ })
        .addDisplacementMarker(() -> { /* grab cone */ })
        .waitSeconds(1)
        .setReversed(false)
        .splineTo(placePose.vec(), placePose.getHeading())
        .addDisplacementMarker(() -> { /* Release Cone  */ })
        .waitSeconds(0.5)
        .addDisplacementMarker(() -> { /* lower lift */})

        // Cycle 4
        .setReversed(true)
        .splineTo(gatherPose.vec(), gatherPose.getHeading())
        .UNSTABLE_addDisplacementMarkerOffset(-1, () -> { /* extend intake */ })
        .addDisplacementMarker(() -> { /* grab cone */ })
        .waitSeconds(1)
        .setReversed(false)
        .splineTo(placePose.vec(), placePose.getHeading())
        .addDisplacementMarker(() -> { /* Release Cone  */ })
        .waitSeconds(0.5)
        .addDisplacementMarker(() -> { /* lower lift */})

        // Cycle 5
        .setReversed(true)
        .splineTo(gatherPose.vec(), gatherPose.getHeading())
        .UNSTABLE_addDisplacementMarkerOffset(-1, () -> { /* extend intake */ })
        .addDisplacementMarker(() -> { /* grab cone */ })
        .waitSeconds(1)
        .setReversed(false)
        .splineTo(placePose.vec(), placePose.getHeading())
        .addDisplacementMarker(() -> { /* Release Cone  */ })
        .waitSeconds(0.5)
        .addDisplacementMarker(() -> { /* lower lift */})

        // Park
        .setReversed(true)
        .splineToLinearHeading(centralParkPose, centralParkPose.getHeading())
        .strafeRight(determination.inches)

        .build();

    drive.followTrajectorySequence(sequence);
  }
}