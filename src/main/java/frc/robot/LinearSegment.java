// change to your package name
package auto;

/**
 * Class containing a heading and distance to drive
 * autonomously. Combine these sequentially to create a path.
 */
public class LinearSegment {

  private double heading, distance;

  public LinearSegment(double heading, double distance) {
    this.heading = heading;
    this.distance = distance;
  }

  public double getHeadingDegrees() {
    return heading;
  }

  public double getDistanceInches() {
    return distance;
  }

}
