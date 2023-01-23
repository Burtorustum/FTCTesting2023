package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Control.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.Structure.Subsystem;

@Config
public class Lift extends Subsystem {

  private static final double COUNTS_PER_REV = 384.5; // PPR
  private static final double RPM = 435;
  private static final double MAX_VELOCITY = RPM * COUNTS_PER_REV / 60; // PPS
  public static PIDFCoefficients coefficients = new PIDFCoefficients(0, 0, 0, 0);
  private final DcMotorEx l1;
  private final PIDFController controller = new PIDFController(coefficients);
  private double velocityTarget;

  protected Lift(LinearOpMode opMode) {
    super(opMode);

    this.l1 = opMode.hardwareMap.get(DcMotorEx.class, "l1");
    this.l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  @Override
  public void update(Telemetry telemetry) {
    double curVelocity = l1.getVelocity();
    double error = velocityTarget - curVelocity;
    double output = controller.update(velocityTarget, error);
    this.l1.setPower(output);

    telemetry.addData("LIFT | velowcity target", velocityTarget);
    telemetry.addData("LIFT | velocity", curVelocity);
    telemetry.addData("LIFT | velocity error", error);
    telemetry.addData("LIFT | controller output", output);
  }

  /**
   * @param target -1 <= target <= 1
   */
  public void setVelocityTarget(double target) {
    this.velocityTarget = target * MAX_VELOCITY;
  }
}