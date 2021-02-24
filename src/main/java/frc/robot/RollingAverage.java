// change this to your package name
package frc.robot;

/**
 * Class for taking a rolling average over a specified number of values
 */
public class RollingAverage {

  private final double[] values;
  private double sum = 0;
  private int i = 0;
  boolean wrapped = false;

  public RollingAverage(int size) {
    this.values = new double[size];
  }

  public void feed(double value) {

    if(Double.isNaN(value) || !Double.isFinite(value)) return;

    sum -= values[i];
    sum += value;
    values[i] = value;
    if(i == values.length - 1) {
      wrapped = true;
    }
    i = (i + 1) % values.length;
  }

  public double avg() {
    if(wrapped) {
      return sum/values.length;
    } else {
      return sum/i;
    }
  }


}