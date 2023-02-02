package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Control.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.Structure.Subsystem;

@Config
public class Lift implements Subsystem {

  // TODO: tune! (first kg, then pid)
  public static PIDFCoefficients coefficients = new PIDFCoefficients(0, 0, 0);
  public static double kg = 0;
  private final PIDFController heightController = new PIDFController(coefficients);

  public static int SHIFT_HEIGHT_TICKS = 300;

  private final DcMotorEx slideTop, slideBottom;
  private final Servo swivel, clawLeft, clawRight;
  private final TouchSensor magLim;

  public enum Height {
    INTAKE(0), GROUND(170), LOW(1097),
    MEDIUM(1650), HIGH(2150);

    public final int encoderTarget;

    Height(int encoderTarget) {
      this.encoderTarget = encoderTarget;
    }

    public int getEncoderTarget(boolean shift) {
      return encoderTarget - (shift ? 300 : 0);
    }
  }

  public enum ClawPosition {
    OPEN(0.51, 0.48), CLOSED(0.912, 0.921);

    public final double leftServoPosition, rightServoPosition;

    ClawPosition(double leftServoPosition, double rightServoPosition) {
      this.leftServoPosition = leftServoPosition;
      this.rightServoPosition = rightServoPosition;
    }
  }

  public enum SwivelPosition {
    IN(0.75), RIGHT(0.35), OUT(0.0);

    public final double servoPosition;

    SwivelPosition(double servoPosition) {
      this.servoPosition = servoPosition;
    }
  }

  private boolean autoLift;
  private Height height;
  private double liftPower; // 0 <= pow <= 1
  private boolean shiftHeight;

  private ClawPosition clawPosition;

  private SwivelPosition swivelPosition; // 0 <= theta <= 180


  public Lift(OpMode opMode) {

    slideTop = opMode.hardwareMap.get(DcMotorEx.class, "slideTop");
    slideBottom = opMode.hardwareMap.get(DcMotorEx.class, "slideBottom");

    // Set the direction of the system motors based on their orientation on the bot
    slideTop.setDirection(DcMotorSimple.Direction.REVERSE);
    slideBottom.setDirection(DcMotorSimple.Direction.REVERSE);

    // Set the zero power behavior for all the system motors
    slideTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Set the mode of the slide motors to run with encoders
    slideTop.setMode(RunMode.STOP_AND_RESET_ENCODER);
    slideBottom.setMode(RunMode.STOP_AND_RESET_ENCODER);

    swivel = opMode.hardwareMap.get(Servo.class, "swivel");
    magLim = opMode.hardwareMap.get(TouchSensor.class, "magLim");
    clawLeft = opMode.hardwareMap.get(Servo.class, "clawLeft");
    clawRight = opMode.hardwareMap.get(Servo.class, "clawRight");

    clawLeft.setPosition(ClawPosition.CLOSED.leftServoPosition);
    clawRight.setPosition(ClawPosition.CLOSED.rightServoPosition);

    this.height = Height.INTAKE;
    this.clawPosition = ClawPosition.CLOSED;
    this.swivelPosition = SwivelPosition.IN;
    this.autoLift = true;
  }

  @Override
  public void start() {
    this.slideTop.setMode(RunMode.RUN_WITHOUT_ENCODER);
    this.slideBottom.setMode(RunMode.RUN_WITHOUT_ENCODER);
  }

  @Override
  public void update(Telemetry telemetry) {
    // Calculate lift encoder avg
    int encoderAvg =
        (this.slideTop.getCurrentPosition() + this.slideBottom.getCurrentPosition()) / 2;
    double encoderError = this.heightError(encoderAvg);

    if (this.autoLift) {
      // Full PID + feed-forward control to go to target height
      if (Math.abs(encoderError) < 20) {
        this.liftPower = 0;
      } else {
        this.liftPower =
            this.heightController.update(height.getEncoderTarget(this.shiftHeight), encoderError)
                + kg;
      }
    } else {
      // Only feed-forward gain for manual control
      this.liftPower = this.liftPower + kg;
    }

    // Prevent lowering if magnetic limit switch detects lift
    if (this.liftPower < 0 && this.magLim.isPressed()) {
      this.liftPower = 0;
    }

    // Prevent lowering to ground / intake if swivel is not going in
    if ((this.height == Height.GROUND || this.height == Height.INTAKE)
        && this.swivel.getPosition() != SwivelPosition.IN.servoPosition) {
      this.liftPower = 0;
    }

    // Prevent high speed when going down
    this.liftPower = Range.clip(this.liftPower, -0.5, 1);

    // Update slide motor powers
    this.slideBottom.setPower(this.liftPower);
    this.slideTop.setPower(this.liftPower);

    // Update claw position
    this.clawLeft.setPosition(this.clawPosition.leftServoPosition);
    this.clawRight.setPosition(this.clawPosition.rightServoPosition);

    // Update swivel position iff the slide is above the low-pole (clears the robot)
    if (encoderAvg >= Height.MEDIUM.encoderTarget
        - SHIFT_HEIGHT_TICKS) { // TODO: possibly needs to be >= something else
      this.swivel.setPosition(this.swivelPosition.servoPosition);
    }
  }

  public void setHeight(Height height) {
    this.height = height;
  }

  public void shiftDown() {
    this.shiftHeight = true;
  }

  public void shiftUp() {
    this.shiftHeight = false;
  }

  public double heightError(int encoderAvg) {
    return this.height.getEncoderTarget(this.shiftHeight) - encoderAvg;
  }

  public void setClawPosition(ClawPosition clawPosition) {
    this.clawPosition = clawPosition;
  }

  public void setSwivelPosition(SwivelPosition swivelPosition) {
    this.swivelPosition = swivelPosition;
  }

  public void toManualLift() {
    this.autoLift = false;
  }

  public void toAutomaticLift() {
    this.autoLift = true;
  }

  public boolean getAutoLift() {
    return this.autoLift;
  }

  public double getError() {
    int encoderAvg =
        (this.slideTop.getCurrentPosition() + this.slideBottom.getCurrentPosition()) / 2;
    return this.heightError(encoderAvg);
  }

  public void manualLiftPower(double liftPower) {
    this.liftPower = liftPower;
  }
}