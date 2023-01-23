package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Control.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.Structure.Subsystem;

@Config
public class MecanumDrivetrain extends Subsystem {

  private static final double COUNTS_PER_REV = 537.7;
  private static final double WHEEL_DIAMETER_MM = 96;
  private static final double DRIVE_REDUCTION = 1;
  private static final double COUNTS_PER_MM = (COUNTS_PER_REV * DRIVE_REDUCTION) /
      (WHEEL_DIAMETER_MM * Math.PI);
  public static PIDFCoefficients turnPIDCoeff = new PIDFCoefficients(2.20, 0.001, 0.07, 0.00);
  public static PIDFCoefficients drivePIDCoeff = new PIDFCoefficients(0.005, 0.00, 0.00, 0.00);
  private final DcMotorEx fl, fr, bl, br;
  private final IMU imu;
  private final PIDFController turnController = new PIDFController(turnPIDCoeff);
  private final PIDFController driveController = new PIDFController(drivePIDCoeff);
  private double flPow, frPow, blPow, brPow;
  private double angleTarget;
  private double distanceTarget;

  private State mode = State.IDLE;

  public MecanumDrivetrain(LinearOpMode opMode, IMU imu) {
    super(opMode);

    fl = hwMap.get(DcMotorEx.class, "fl");
    fr = hwMap.get(DcMotorEx.class, "fr");
    bl = hwMap.get(DcMotorEx.class, "bl");
    br = hwMap.get(DcMotorEx.class, "br");

    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    fr.setDirection(DcMotorSimple.Direction.FORWARD);
    bl.setDirection(DcMotorSimple.Direction.REVERSE);
    br.setDirection(DcMotorSimple.Direction.FORWARD);

    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    this.imu = imu;
  }

  public void update(Telemetry telemetry) {
    telemetry.addData("Drivetrain | Mode", this.mode.name());
    switch (mode) {
      case IDLE:
        flPow = 0;
        frPow = 0;
        blPow = 0;
        brPow = 0;
        break;

      case TURN:
        double botAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double angleError = -AngleUnit.normalizeRadians(
            AngleUnit.DEGREES.toRadians(this.angleTarget) - botAngle);

        double turnOutput = turnController.update(this.angleTarget, angleError);
        flPow = turnOutput;
        blPow = turnOutput;
        frPow = -turnOutput;
        brPow = -turnOutput;

        // TODO: is this the correct value?
        if (turnOutput <= 0.01) {
          this.mode = State.IDLE;
        }

        telemetry.addData("Drivetrain | Bot Angle", Math.toDegrees(botAngle));
        telemetry.addData("Drivetrain | Target Angle", angleTarget);
        telemetry.addData("Drivetrain | Angle Error", Math.toDegrees(angleError));
        telemetry.addData("Drivetrain | Turn Output", turnOutput);
        break;

      case DRIVE_DISTANCE:
        double botDistance = (fl.getCurrentPosition()
            + fr.getCurrentPosition()
            + bl.getCurrentPosition()
            + br.getCurrentPosition()) / 4.0;
        double distanceError = distanceTarget - botDistance;

        double driveOutput = driveController.update(this.distanceTarget, distanceError);
        flPow = driveOutput;
        blPow = driveOutput;
        frPow = driveOutput;
        brPow = driveOutput;

        // TODO: is this the correct value?
        if (driveOutput <= 0.01) {
          this.mode = State.IDLE;
        }

        telemetry.addData("Drivetrain | Bot Distance", botDistance);
        telemetry.addData("Drivetrain | Target Distance", distanceTarget);
        telemetry.addData("Drivetrain | Distance Error", distanceError);
        telemetry.addData("Drivetrain | Drive Output", driveOutput);
        break;

      case DIRECT_CONTROL:
        break;
    }

    fl.setPower(flPow);
    fr.setPower(frPow);
    bl.setPower(blPow);
    br.setPower(brPow);
  }

  public void turn(int targetDegrees) {
    this.angleTarget = targetDegrees;
    this.mode = State.TURN;
  }

  public void driveDistance(int distanceTarget, DistanceUnit unit) {
    double targetMM = unit.toMm(distanceTarget);
    this.distanceTarget = targetMM * COUNTS_PER_MM;
    this.mode = State.DRIVE_DISTANCE;
  }

  public boolean driveComplete() {
    return this.mode == State.IDLE;
  }

  // TODO: test
  public void zeroMotorEncoders() throws LynxNackException, InterruptedException {
    List<LynxModule> hubs = this.hwMap.getAll(LynxModule.class);
    for (LynxModule hub : hubs) {
      if (hub.isParent()) {
        new LynxResetMotorEncoderCommand(hub, fl.getPortNumber()).send();
        new LynxResetMotorEncoderCommand(hub, bl.getPortNumber()).send();
      } else {
        new LynxResetMotorEncoderCommand(hub, fr.getPortNumber()).send();
        new LynxResetMotorEncoderCommand(hub, br.getPortNumber()).send();
      }
    }
  }

  // TODO: test
  public boolean isEncoderResetFinished() {
    return fl.getCurrentPosition() == 0
        && fr.getCurrentPosition() == 0
        && bl.getCurrentPosition() == 0
        && br.getCurrentPosition() == 0;
  }

  public void directControl(double flPow, double frPow, double blPow, double brPow) {
    this.mode = State.DIRECT_CONTROL;
    this.flPow = flPow;
    this.frPow = frPow;
    this.blPow = blPow;
    this.brPow = brPow;
  }

  public void fieldCentric(double y, double x, double rx) {
    // Read inverse IMU heading, as the IMU heading is CW positive
    double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
    double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double flPow = (rotY + rotX + rx) / denominator;
    double frPow = (rotY - rotX - rx) / denominator;
    double blPow = (rotY - rotX + rx) / denominator;
    double brPow = (rotY + rotX - rx) / denominator;

    this.directControl(flPow, frPow, blPow, brPow);
  }

  enum State {
    IDLE, TURN, DRIVE_DISTANCE, DIRECT_CONTROL
  }
}
