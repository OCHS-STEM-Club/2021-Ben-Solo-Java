package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class specific to your robot that contains all drive code to follow a path
 * consisting of segments, each with heading and distance.
 */
public class BenSoloPathDriver extends PathDriver {

  // add any fields needed by this class here

  private RadialDrive drive;
  private PIDController turningController, driveController, radiusController;
  private BenSoloMotorSetup motorSetup;
  private AHRS navx;

  public BenSoloPathDriver(
      BenSoloMotorSetup motorSetup,
      AHRS navx,
      PIDController turningController,
      PIDController driveController,
      PIDController radiusController,
      RadialDrive drive
  ) {
    this.motorSetup = motorSetup;
    this.turningController = turningController;
    this.driveController = driveController;
    this.radiusController = radiusController;
    this.navx = navx;
    this.drive = drive;
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

    turningController.setTarget(headingDegrees);
    radiusController.setTarget(headingDegrees);

  }

  /**
   * This method is called periodically while the robot turns to a new target heading.
   *
   * @param headingDegrees the target heading in degrees
   * @param i              the index of the current segment (starts at 0)
   * @return true if the robot is done turning, false otherwise
   */
  @Override protected boolean turnPeriodic(double headingDegrees, int i) {

    double turnError = headingDegrees - navx.getAngle();

    SmartDashboard.putNumber("turn error", turnError);

    double turnSpeed = turningController.getControlOutput(navx.getAngle());

    boolean left = turnSpeed < 0;

    SmartDashboard.putBoolean("turn left", left);
    SmartDashboard.putNumber("turn speed", turnSpeed);

    // TODO: fix negative radius
    drive.radialDrive(left, 0, Math.abs(turnSpeed), false);

    return turningController.atTarget();

  }

  /**
   * This method is called ONCE when the robot starts to drive a linear distance.
   *
   * @param distanceInches the target distance in inches
   * @param i              the index of the current segment (starts at 0)
   */
  @Override protected void driveInit(double distanceInches, int i) {

    driveController.setTarget(distanceInches);

    motorSetup.getLeftCanEncoder().setPosition(0);
    motorSetup.getRightCanEncoder().setPosition(0);

  }

  /**
   * This method is called periodically while the robot drives a set distance forward.
   *
   * @param distanceInches the target distance in inches
   * @param i              the index of the current segment (starts at 0)
   * @return true if the robot is done turning, false otherwise
   */
  @Override protected boolean drivePeriodic(double distanceInches, int i) {

    double radOutput = radiusController.getControlOutput(navx.getAngle());

    SmartDashboard.putNumber("drive radius output", radOutput);

    double radius = Math.abs(Utils.getRadius(radOutput));

    double driveOutput = driveController.getControlOutput(motorSetup.getLeftPositionInches());

    drive.radialDrive(output < 0, radius, driveOutput, false);

    return driveController.atTarget();

  }

  /**
   * This method is called ONCE when all segments are driven. Place any code
   * needed to stop all motors in this method.
   */
  @Override protected void pathStop() {

    drive.radialDrive(false, 0, 0);

  }

}
