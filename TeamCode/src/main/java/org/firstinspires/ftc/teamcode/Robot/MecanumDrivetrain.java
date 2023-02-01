package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
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
  public static double STRAFE_MULTIPLIER = 1.0;

  private final DcMotorEx fl, fr, bl, br;
  private final IMU imu;

  public static PIDFCoefficients turnPIDCoeff = new PIDFCoefficients(2.20, 0.001, 0.07, 0.00);
  private final PIDFController turnController = new PIDFController(turnPIDCoeff);

  public static PIDFCoefficients drivePIDCoeff = new PIDFCoefficients(0.005, 0.00, 0.00, 0.00);
  private final PIDFController flController = new PIDFController(drivePIDCoeff);
  private final PIDFController frController = new PIDFController(drivePIDCoeff);
  private final PIDFController blController = new PIDFController(drivePIDCoeff);
  private final PIDFController brController = new PIDFController(drivePIDCoeff);

  private double flPow, frPow, blPow, brPow;
  private double angleTarget;
  private int flTarget, frTarget, blTarget, brTarget;

  private State mode = State.IDLE;

  public MecanumDrivetrain(LinearOpMode opMode, IMU imu) {
    super(opMode);

    fl = hwMap.get(DcMotorEx.class, "FL");
    fr = hwMap.get(DcMotorEx.class, "FR");
    bl = hwMap.get(DcMotorEx.class, "BL");
    br = hwMap.get(DcMotorEx.class, "BR");

    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    fr.setDirection(DcMotorSimple.Direction.FORWARD);
    bl.setDirection(DcMotorSimple.Direction.REVERSE);
    br.setDirection(DcMotorSimple.Direction.FORWARD);

    fl.setMode(RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(RunMode.STOP_AND_RESET_ENCODER);
    bl.setMode(RunMode.STOP_AND_RESET_ENCODER);
    br.setMode(RunMode.STOP_AND_RESET_ENCODER);

    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    this.imu = imu;
  }

  @Override
  public void start() {
    fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(RunMode.RUN_WITHOUT_ENCODER);
  }

  enum State {
    IDLE, TURN, DRIVE_DISTANCE, DIRECT_CONTROL
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

        if (turnOutput <= 0.01) {
          this.mode = State.IDLE;
        }

        telemetry.addData("Drivetrain | Bot Angle", Math.toDegrees(botAngle));
        telemetry.addData("Drivetrain | Target Angle", angleTarget);
        telemetry.addData("Drivetrain | Angle Error", Math.toDegrees(angleError));
        telemetry.addData("Drivetrain | Turn Output", turnOutput);
        break;

      case DRIVE_DISTANCE:
        int flError = this.flTarget - this.fl.getCurrentPosition();
        int frError = this.frTarget - this.fr.getCurrentPosition();
        int blError = this.blTarget - this.bl.getCurrentPosition();
        int brError = this.brTarget - this.br.getCurrentPosition();

        flPow = flController.update(this.flTarget, flError);
        frPow = frController.update(this.brTarget, frError);
        blPow = blController.update(this.brTarget, blError);
        brPow = brController.update(this.brTarget, brError);

        if ((flPow + frPow + blPow + brPow) / 4.0 <= 0.01) {
          this.mode = State.IDLE;
        }

        telemetry.addData("Drivetrain | Average Error",
            (flError + frError + blError + brError) / 4.0);
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
    int targetEncoders = (int) (unit.toMm(distanceTarget) * COUNTS_PER_MM);
    this.updateEncoderTargets(targetEncoders, targetEncoders, targetEncoders, targetEncoders);

    this.mode = State.DRIVE_DISTANCE;
  }

  public void strafeDistance(int distanceTarget, DistanceUnit unit) {
    int targetEncoders = (int) (unit.toMm(distanceTarget * STRAFE_MULTIPLIER) * COUNTS_PER_MM);
    this.updateEncoderTargets(targetEncoders, -targetEncoders, -targetEncoders, targetEncoders);

    this.mode = State.DRIVE_DISTANCE;
  }

  private void updateEncoderTargets(int flTarget, int frTarget, int blTarget, int brTarget) {
    this.flTarget = flTarget + this.fl.getCurrentPosition();
    this.frTarget = frTarget + this.fr.getCurrentPosition();
    this.blTarget = blTarget + this.bl.getCurrentPosition();
    this.brTarget = brTarget + this.br.getCurrentPosition();
  }

  public boolean driveComplete() {
    return this.mode == State.IDLE;
  }

  public boolean isEncoderResetFinished() {
    return fl.getCurrentPosition() == 0
        && fr.getCurrentPosition() == 0
        && bl.getCurrentPosition() == 0
        && br.getCurrentPosition() == 0;
  }

  public void setMode(DcMotor.RunMode mode) {
    this.fl.setMode(mode);
    this.fr.setMode(mode);
    this.bl.setMode(mode);
    this.br.setMode(mode);
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
}