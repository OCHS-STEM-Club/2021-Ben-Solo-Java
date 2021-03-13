package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract superclass for path driving. This class contains all logic to
 * progress through each segment in the drive path.
 */
public abstract class RadialPathDriver {

  private int segmentIndex = 0;
  private RadialSegment[] radialSegments; // multiple segments make up a path totally not obvious
  private boolean stopped = false;

  

  // implement this method in your extension of this class
  protected abstract void pathInit();

  // implement this method in your extension of this class
  protected abstract void arcInit(double headingDegrees, double radius, boolean left, int i);

  // implement this method in your extension of this class
  protected abstract boolean arcPeriodic(double headingDegrees, double radius, boolean left, int i);

  // implement this method in your extension of this class
  protected abstract void straightInit(double headingDegrees, double distanceInches, int i);

  // implement this method in your extension of this class
  protected abstract boolean straightPeriodic(double headingDegrees, double distanceInches, int i);

  // implement this method in your extension of this class
  protected abstract void pathStop();
  
  //get the number fo segments in the path
  int getPathSegmentCount() {
    return radialSegments == null ? 0 : radialSegments.length;
  }

  //loads the path
  public void loadSegments(RadialSegment... radialSegments) {
    this.radialSegments = radialSegments;
  }

  /**
   * Called in autonomousInit()
   * 
   * prepares the robot to run the whole path
   */
  public void init() {

    segmentIndex = 0;

    pathInit();

    if (radialSegments == null || radialSegments.length == 0) {

      System.out.println("No path driver segments loaded!");

    } else {

      initSegment(0);

    }

  }

  private void initSegment(int i) {

    RadialSegment segment = radialSegments[i]; // getting the desired segment

    // prepearing robot for next action
    if (segment.isTurn()) {
      arcInit(segment.getHeadingDegrees(), segment.getRadius(), segment.isTurningLeft(), i); //uses navx
    } else {
      straightInit(segment.getHeadingDegrees(), segment.getDistanceInches(), i); //uses encoders
    }

  }

  /**
   * Called in autonomousPeriodic()
   * 
   * drives robot for current segment in the path
   */
  public void periodic() {

    SmartDashboard.putNumber("auto index", segmentIndex);

    if (radialSegments == null || segmentIndex >= radialSegments.length) { // if path is done or if there is no path, the robot stops

      if (!stopped) { // code stops the robot ONCEEE

        pathStop();

        stopped = true;

      }

    } else { // drives current segment

      RadialSegment segment = radialSegments[segmentIndex];

      if (segment.isTurn()) { // segment is a turn

        if (arcPeriodic(segment.getHeadingDegrees(), segment.getRadius(), segment.isTurningLeft(), segmentIndex)) {

          // just finished turn

          segmentIndex++;

          if (segmentIndex < radialSegments.length) {

            initSegment(segmentIndex); // initializing for the next segment

          }

        }

      } else { // segment is straight

        if (straightPeriodic(segment.getHeadingDegrees(), segment.getDistanceInches(), segmentIndex)) {
          
          //just finished going straight

          segmentIndex++;

          if (segmentIndex < radialSegments.length) {

            initSegment(segmentIndex); // initializing for the next segment

          }

        }

      }

    }

  }

}
