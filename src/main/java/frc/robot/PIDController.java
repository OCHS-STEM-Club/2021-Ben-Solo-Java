package frc.robot;

/**
 * @author Jacob Schramkowski
 *
 *         A simple implementation of a Proportional-Integral-Derivative (PID)
 *         Controller designed for single-line use
 */
public class PIDController {

  // tuning parameters
  private double Kp, Ki, Kd;

  // required fields for tracking error over time
  private double prevError = 0;
  private double integral = 0;
  private long prevTime = System.nanoTime();

  private int stopCounter = 0;
  private int maxStopCounter = 18;

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
   * @param error the signed distance from the current to desired position
   * @return the control output
   */
  public double getValue(double error) {

    // compute change in time since last getValue() call in seconds
    double timeDelta = (System.nanoTime() - prevTime) / 1e9;

    // add integral error
    integral += prevError * timeDelta;

    // compute derivative error
    double derivative = (error - prevError) / timeDelta;

    // sum all error terms with respective control factors to get control output
    double value = Kp * error + Ki * integral + Kd * derivative;

    // if there is a set minimum output, apply it
    if (!Double.isNaN(min)) {
      value = Math.max(value, min);
    }

    // if there is a set maximum output, apply it
    if (!Double.isNaN(max)) {
      value = Math.min(value, max);
    }

    if (Math.abs((prevError - error) / prevError) < 0.01) {
      stopCounter++;
    } else {
      stopCounter = 0;
    }

    // set the previous error field equal to the current error for the next loop
    prevError = error;

    // set the previous timestamp field equal to the current timestamp for the next
    // loop
    prevTime = System.nanoTime();

    return value;

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

  public void setKp(double kp) {
    Kp = kp;
  }

  public void setKi(double ki) {
    Ki = ki;
  }

  public void setKd(double kd) {
    Kd = kd;
  }

  public boolean atTarget() {
    if (stopCounter > maxStopCounter) {
      stopCounter = 0;
      return true;
    } else {
      return false;
    }
  }

  @Override
  public String toString() {
    return String.format("Kp = %f, Ki = %f, Kd = %f", Kp, Ki, Kd);
  }

}
