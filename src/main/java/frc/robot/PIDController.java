// change this to your package name
package frc.robot;

/**
 * @author Jacob Schramkowski
 *         <p>
 *         A simple implementation of a Proportional-Integral-Derivative (PID)
 *         Controller designed for drop-in use
 */
public class PIDController {

  // tuning parameters
  private double Kp, Ki, Kd;

  // required fields for tracking error over time
  private double prevError = 0;
  private double errorIntegral = 0;
  private long prevTime = System.nanoTime();

  // field for set end point
  private double target = 0;

  // fields for detecting when the object is at its target position
  private double stopThreshold = 1;
  private int stopCount = 10;
  private int currCount = 0;

  // optional min and max output values, default to NaN if unused
  private double min = Double.NaN, max = Double.NaN;

  /**
   * Constructor for PID Controller object
   *
   * @param Kp the proportional control factor
   * @param Ki the integral control factor
   * @param Kd the derivative control factor
   */
  public PIDController(double Kp, double Ki, double Kd) {
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
  }

  /**
   * Gets the value of the control output given the distance from the current
   * position to the desired position
   *
   * @param position the signed current position of the controlled object
   * @return the control output value
   */
  public double getControlOutput(double position) {

    double error = target - position;

    // compute change in time since last getValue() call in seconds
    double timeDelta = (System.nanoTime() - prevTime) / 1e9;

    // add integral error
    errorIntegral += prevError * timeDelta;

    // compute derivative error
    double errorDerivative = (error - prevError) / timeDelta;

    // sum all error terms with respective control factors to get control output
    double value = Kp * error + Ki * errorIntegral + Kd * errorDerivative;

    // if there is a set minimum output, apply it
    if (!Double.isNaN(min)) {
      value = Math.max(value, min);
    }

    // if there is a set maximum output, apply it
    if (!Double.isNaN(max)) {
      value = Math.min(value, max);
    }

    // if the absolute value of the error is within the acceptable threshold,
    // increment the count, else reset it
    if (Math.abs(error) < stopThreshold) {
      currCount++;
    } else {
      currCount = 0;
    }

    // set the previous error field equal to the current error for the next loop
    prevError = error;

    // set the previous timestamp field equal to the current timestamp for the next
    // loop
    prevTime = System.nanoTime();

    return value;

  }

  /**
   * Sets the desired end point for the controlled object
   *
   * @param target the target point in the same units as the value fed into
   *               getControlOutput()
   */
  public void setTarget(double target) {
    this.target = target;
    currCount = 0;
    errorIntegral = 0;
  }

  /**
   * Gets the currently set target point
   *
   * @return the target point
   */
  public double getTarget() {
    return target;
  }

  /**
   * Checks whether the controlled object is at its desired end point
   *
   * @return true if the object has been within an acceptable range for a
   *         specified number of consecutive cycles
   */
  public boolean atTarget() {
    return currCount > stopCount;
  }

  /**
   * Sets the conditions for when atTarget() returns true
   *
   * @param threshold the acceptable error above or below the target point
   * @param count     the number of consecutive times the position must fall
   *                  within the above range
   */
  public void setTargetDetection(double threshold, int count) {
    this.stopThreshold = threshold;
    this.stopCount = count;
  }

  /**
   * Sets the minimum control output value
   *
   * @param min the minimum value of the control output
   */
  public void setMin(double min) {
    this.min = min;
  }

  /**
   * Sets the maximum control output value
   *
   * @param max the maximum value of the control output
   */
  public void setMax(double max) {
    this.max = max;
  }

  @Override
  public String toString() {
    return String.format("Kp = %f, Ki = %f, Kd = %f", Kp, Ki, Kd);
  }

  public void setKp(double kp) {
    Kp = kp;
  }

  public void setKi(double ki) {
    Ki = ki;
  }

  public void setKd(double kd) {
    Kd = kd;
  }

  public double getKp() {
    return Kp;
  }

  public double getKi() {
    return Ki;
  }

  public double getKd() {
    return Kd;
  }

}