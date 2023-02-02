package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

/*
 * Sample tracking wheel localizer implementation assuming the standard configur    ation:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {

  public static double TICKS_PER_REV = 8192;
  public static double WHEEL_RADIUS = 0.6889764; // in
  public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

  // TODO: retune lat dist and forward offset
  public static double LATERAL_DISTANCE = 13.951; // in; distance between the left and right odom wheels
  public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;
  private final Encoder frontEncoder;

  // TODO: retune x/y mult
  double X_MULT = 1.0344;
  double Y_MULT = 1.0239;

  public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
    super(Arrays.asList(
        new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
        new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
        new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
    ));

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "l"));
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "r"));
    frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "m"));

    frontEncoder.setDirection(Encoder.Direction.REVERSE);
  }

  public static double encoderTicksToInches(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() {
    return Arrays.asList(
        encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULT,
        encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULT,
        encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULT
    );
  }

  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
    return Arrays.asList(
        encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULT,
        encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULT,
        encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULT
    );
  }
}