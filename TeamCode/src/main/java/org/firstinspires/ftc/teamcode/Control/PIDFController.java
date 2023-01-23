package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFController {

  private final ElapsedTime timer;
  public PIDFCoefficients coeff;
  private double lastTarget, integralSum, lastError, previousFilterEstimate;

  public PIDFController(PIDFCoefficients coeff) {
    this.coeff = coeff;
    this.lastTarget = 0;
    this.integralSum = 0;
    this.lastError = 0;
    this.previousFilterEstimate = 0;
    this.timer = new ElapsedTime();
  }

  public double update(double target, double error) {
    // calculate error delta
    double errorChange = (error - lastError);

    // derivative -- rate of change of the error
    // filter out height frequency noise to increase derivative performance (low pass filter)
    double derivative;
    if (coeff.lowPassGain > 0 && coeff.lowPassGain < 1) {
      double currentFilterEstimate =
          (coeff.lowPassGain * previousFilterEstimate) + (1 - coeff.lowPassGain) * errorChange;
      previousFilterEstimate = currentFilterEstimate;
      derivative = currentFilterEstimate / timer.seconds();
    } else {
      derivative = (error - lastError) / timer.seconds();
    }

    // sum of error over time
    integralSum += (error * timer.seconds());

    // max out integral sum (integral windup mitigation)
    integralSum = Range.clip(integralSum, -coeff.maxIntegralSum, coeff.maxIntegralSum);

    // reset integral sum if target changes
    if (target != lastTarget) {
      integralSum = 0;
    }

    lastError = error;
    lastTarget = target;

    // reset the timer for next time
    timer.reset();

    return (coeff.kp * error) + (coeff.ki * integralSum) + (coeff.kd * derivative) + (coeff.kf
        * target);
  }
}

