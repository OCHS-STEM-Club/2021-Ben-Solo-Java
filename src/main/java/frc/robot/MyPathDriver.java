// change to your package name
package auto;

/**
 * Class specific to your robot that contains all drive code to follow a path
 * consisting of segments, each with heading and distance.
 */
public class MyPathDriver extends PathDriver {

  // add any fields needed by this class here

  public MyPathDriver(/* add any constructor arguments here */) {

    // set all fields in your constructor here

  }

  /**
   * This method is called when init() is called on
   * an instance of this class.
   */
  @Override protected void pathInit() {

    // place any initialization code here

  }

  /**
   * This method is called ONCE when the robot is starting to turn to a new heading.
   *
   * @param headingDegrees the target heading in degrees
   * @param i              the index of the current segment (starts at 0)
   */
  @Override protected void turnInit(double headingDegrees, int i) {

    // add any turn initialization here

  }

  /**
   * This method is called periodically while the robot turns to a new target heading.
   *
   * @param headingDegrees the target heading in degrees
   * @param i              the index of the current segment (starts at 0)
   * @return true if the robot is done turning, false otherwise
   */
  @Override protected boolean turnPeriodic(double headingDegrees, int i) {

    // add your turn code here

    // return true when this segment is completed
    return false;

  }

  /**
   * This method is called ONCE when the robot starts to drive a linear distance.
   *
   * @param distanceInches the target distance in inches
   * @param i              the index of the current segment (starts at 0)
   */
  @Override protected void driveInit(double distanceInches, int i) {

    // add any drive initialization here

  }

  /**
   * This method is called periodically while the robot drives a set distance forward.
   *
   * @param distanceInches the target distance in inches
   * @param i              the index of the current segment (starts at 0)
   * @return true if the robot is done turning, false otherwise
   */
  @Override protected boolean drivePeriodic(double distanceInches, int i) {

    // add your drive code here

    // return true when this segment is completed
    return false;

  }

  /**
   * This method is called ONCE when all segments are driven. Place any code
   * needed to stop all motors in this method.
   */
  @Override protected void pathStop() {

    // stop your robot here

  }

}
