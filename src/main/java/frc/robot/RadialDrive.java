package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RadialDrive {

  // data points for speed vs voltage
  // 0V = 0 in/s
  // 1V = 13.5 in/s
  // 1.8V = 30 in/s
  // 2.5V = 44 in/s
  // 3.6V = 66.5 in/s
  // 5V = 95 in/s

  private static final double ROBOT_WIDTH = 17.5;
  private static final double SPEED_LIMIT = 1;
  private static final double MAX_VOLTAGE = 7; // ~135 in/s max speed

  public static final double STRAIGHT_RADIUS = 1e12;

  private SpeedController leftGroup, rightGroup;
  private BenSoloMotorSetup motorSetup;

  public RadialDrive(BenSoloMotorSetup motorSetup) {
    this.motorSetup = motorSetup;
    this.leftGroup = motorSetup.getLeftMotorController();
    this.rightGroup = motorSetup.getRightMotorController();
  }

  public void radialDrive(boolean left, double radius, double forwardAxis) {
    radialDrive(left, radius, forwardAxis, true);
  }

  public void radialDrive(boolean left, double radius, double forwardAxis, boolean limit) {

    if (radius < 0) {
      System.out.println("Radius cannot be negative: " + radius);
    }

    radius = Math.abs(radius);

    double leftPos = -ROBOT_WIDTH / 2;
    double rightPos = ROBOT_WIDTH / 2;

    double leftProp = radius - leftPos;
    double rightProp = radius - rightPos;

    // scale both props down by the maximum magnitude (one will always be length 1)
    double max = Math.max(Math.abs(leftProp), Math.abs(rightProp));
    leftProp *= forwardAxis / max;
    rightProp *= forwardAxis / max;

    if (limit) {
      leftProp *= SPEED_LIMIT;
      rightProp *= SPEED_LIMIT;
    }

    // swapping the left and right prop values to turn left
    if (left) {
      double tempLeftProp = leftProp;
      leftProp = rightProp;
      rightProp = tempLeftProp;
    }

    SmartDashboard.putNumber("leftProp", leftProp);
    SmartDashboard.putNumber("rightProp", rightProp);

    // set motor voltages proportional to maximum motor voltage
    leftGroup.setVoltage(leftProp * MAX_VOLTAGE);
    rightGroup.setVoltage(-rightProp * MAX_VOLTAGE);

  }

}