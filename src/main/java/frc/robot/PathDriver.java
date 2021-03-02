package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract superclass for path driving. This class contains all
 * logic to progress through each segment in the drive path.
 */
public abstract class PathDriver {

  private int segmentIndex = 0;
  private LinearSegment[] linearSegments;
  private boolean stopped = false;
  private boolean turning = true;

  // implement this method in your extension of this class
  protected abstract void pathInit();

  // implement this method in your extension of this class
  protected abstract void turnInit(double headingDegrees, int i);

  // implement this method in your extension of this class
  protected abstract boolean turnPeriodic(double headingDegrees, int i);

  // implement this method in your extension of this class
  protected abstract void driveInit(double distanceInches, int i);

  // implement this method in your extension of this class
  protected abstract boolean drivePeriodic(double distanceInches, int i);

  // implement this method in your extension of this class
  protected abstract void pathStop();

  int getPathSegmentCount() {
    return linearSegments == null ? 0 : linearSegments.length;
  }

  public void loadSegments(LinearSegment... linearSegments) {
    this.linearSegments = linearSegments;
  }

  public void init() {

    segmentIndex = 0;

    pathInit();

    if (linearSegments == null || linearSegments.length == 0) {

      System.out.println("No path driver segments loaded!");

    } else {

      turnInit(linearSegments[0].getHeadingDegrees(), 0);

    }

  }

  public void periodic() {

    SmartDashboard.putNumber("auto index", segmentIndex);

    if (linearSegments == null || segmentIndex >= linearSegments.length) {

      if (!stopped) {

        pathStop();

        stopped = true;

      }

    } else if (turning) {

      LinearSegment segment = linearSegments[segmentIndex];

      double heading = segment.getHeadingDegrees();

      if (turnPeriodic(heading, segmentIndex)) {

        turning = false;

        driveInit(segment.getDistanceInches(), segmentIndex);

      }

    } else {

      double distance = linearSegments[segmentIndex].getDistanceInches();

      if (drivePeriodic(distance, segmentIndex)) {

        segmentIndex++;

        if (segmentIndex < linearSegments.length) {

          turning = true;

          turnInit(linearSegments[segmentIndex].getHeadingDegrees(), segmentIndex);

        }

      }

    }

  }

}
