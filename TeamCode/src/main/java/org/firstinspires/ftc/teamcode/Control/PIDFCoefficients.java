package org.firstinspires.ftc.teamcode.Control;

public class PIDFCoefficients {

  public double kp, ki, kd, kf, maxIntegralSum, lowPassGain;

  /**
   * Creating a full PIDF controller with windup-prevention and derivative filtering
   *
   * @param kp             The proportional term
   * @param ki             The integral term
   * @param kd             The derivative term
   * @param kf             The feedforward term
   * @param maxIntegralSum The maximum integral sum for windup prevention (Recommended that for FTC
   *                       maxIntegralSum * ki ~= 0.25)
   * @param lowPassGain    The filtering value in the range 0 < a < 1
   */
  public PIDFCoefficients(double kp, double ki, double kd, double kf, double maxIntegralSum,
      double lowPassGain) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.kf = kf;
    this.maxIntegralSum = maxIntegralSum;
    this.lowPassGain = lowPassGain;
  }

  /**
   * for creating a full PID Controller with feed-forward
   *
   * @param kp The proportional term
   * @param ki The integral term
   * @param kd The derivative term
   * @param kf The feedforward term
   */
  public PIDFCoefficients(double kp, double ki, double kd, double kf) {
    // Setting maxIntegralSum to max int val essentially disables it
    // Setting lowPassGain to 0 disables it as the valid range is 0 < a < 1
    this(kp, ki, kd, kf, Integer.MAX_VALUE - 1, 0);
  }

  /**
   * For creating a simple PID Controller
   *
   * @param kp The proportional term
   * @param ki The integral term
   * @param kd The derivative term
   */
  public PIDFCoefficients(double kp, double ki, double kd) {
    this(kp, ki, kd, 0);
  }

  /**
   * For creating a zeroed-out controller
   */
  public PIDFCoefficients() {
    this(0, 0, 0);
  }

  /**
   *
   */
  public void autoSetMaxIntegralSum() {
    if (this.ki != 0) {
      this.maxIntegralSum = 0.25 / this.ki;
    }
  }
}
