// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.Util;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private AnalogPotentiometer pot = new AnalogPotentiometer(1);

  private BenSoloMotorSetup motorSetup = new BenSoloMotorSetup();

  private XboxController controller = new XboxController(0);

  private RadialDrive radialDrive = new RadialDrive(motorSetup.getLeftMotorController(),
      motorSetup.getRightMotorController());

  private AHRS navx = new AHRS();

  private double startingAngle = 0;

  private boolean turningFlag = false;

  private PIDController turningController = new PIDController(0.015, 0, 0.0031);
  private PIDController driveController = new PIDController(0.019, 0, 0.003);
  private PIDController radiusController = new PIDController(0.7, 0, 0.2);

  private Path path = new Path(motorSetup, navx, turningController, driveController, radialDrive);

  private ContinuousPath continuousPath = new ContinuousPath(motorSetup, navx, turningController, driveController,
      radiusController, radialDrive);

  public Robot() {

    turningController.setMin(-0.6);
    turningController.setMax(0.6);
    turningController.setTargetDetection(5, 20);

    driveController.setMin(-0.6);
    driveController.setMax(0.6);
    driveController.setTargetDetection(5, 20);

    radiusController.setMin(-0.75);
    radiusController.setMax(0.75);
    // driveController.setTargetDetection(5, 20);

    path.addSegments(GeneratedPath.MAIN);
    continuousPath.addSegments(SavedPaths.MAIN);

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // startingAngle = navx.getAngle();
    // navx.setAngleAdjustment(-startingAngle);

    navx.zeroYaw();

    SmartDashboard.putNumber("Kp", radiusController.getKp());
    SmartDashboard.putNumber("Ki", radiusController.getKi());
    SmartDashboard.putNumber("Kd", radiusController.getKd());

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    double potValue = pot.get();

    SmartDashboard.putNumber("potValue", potValue);

    SmartDashboard.putNumber("left encoder", motorSetup.getLeftEncoderInches());
    SmartDashboard.putNumber("right encoder", motorSetup.getRightEncoderInches());
    SmartDashboard.putNumber("heading", navx.getAngle());

    double vLeft = motorSetup.getLeftCanEncoder().getVelocity();
    double vRight = motorSetup.getRightCanEncoder().getVelocity();
    double leftOverRight = vLeft / vRight;

    SmartDashboard.putNumber("left RPM", vLeft);
    SmartDashboard.putNumber("right RPM", vRight);
    SmartDashboard.putNumber("LR RPM ratio", leftOverRight);

    radiusController.setKp(SmartDashboard.getNumber("Kp", 0));
    radiusController.setKi(SmartDashboard.getNumber("Ki", 0));
    radiusController.setKd(SmartDashboard.getNumber("Kd", 0));

    if (controller.getRawButton(4)) {

      // navx.setAngleAdjustment(-navx.getAngle());

      // navx.reset();

      navx.zeroYaw();

      motorSetup.getLeftCanEncoder().setPosition(0);
      motorSetup.getRightCanEncoder().setPosition(0);

    }

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    // startingAngle = navx.getAngle();
    // navx.setAngleAdjustment(-startingAngle);
    navx.zeroYaw();

    motorSetup.getLeftCanEncoder().setPosition(0);
    motorSetup.getRightCanEncoder().setPosition(0);

    // path.initDrive();
     continuousPath.initDrive();

    //radiusController.setTarget(0);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    //path.autoDrive();
    continuousPath.autoDrive();

   /* double output = radiusController.getControlOutput(navx.getAngle());

    double radius = Utils.getRadius(output);

    SmartDashboard.putNumber("radius", radius);
    SmartDashboard.putNumber("turnOutput", output);

    radialDrive.radialDrive(radius, 0.35, false);*/

    /*
     * double speed = driveController.getValue(120 -
     * motorSetup.getLeftEncoderInches());
     * 
     * radialDrive.radialDrive(RadialDrive.STRAIGHT_RADIUS, speed);
     */

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // startingAngle = navx.getAngle();
    // navx.setAngleAdjustment(-startingAngle);
    navx.zeroYaw();

    turningFlag = false;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double forwardAxis = controller.getRawAxis(3) - controller.getRawAxis(2);

    // forwardAxis = Utils.scaleAxis(forwardAxis);

    if (controller.getRawButton(1)) {
      forwardAxis = 0.5 * forwardAxis;
    }

    double turnAxis = controller.getRawAxis(0);
    double radius = Utils.getRadius(turnAxis);

    if (controller.getPOV() == 270) {
      radius = -24;

    } else if (controller.getPOV() == 90) {
      radius = 24;
    }

    if (controller.getRawButton(2)) {
      motorSetup.getLeftCanEncoder().setPosition(0);
      motorSetup.getRightCanEncoder().setPosition(0);
    }

    if (controller.getRawButton(3)) {
      turningFlag = false;
    }

    if (controller.getRawButton(6)) {
      turningFlag = true;
      turningController.setTarget(navx.getAngle() + 90);
    } else

    if (controller.getRawButton(5)) {
      turningFlag = true;
      turningController.setTarget(navx.getAngle() - 90);
    } else

    if (turningFlag) {

      double speed = turningController.getControlOutput(navx.getAngle());

      radialDrive.radialDrive(0, speed, false);

      if (turningController.atTarget()) {
        turningFlag = false;
      }

    } else {
      radialDrive.radialDrive(radius, forwardAxis);
    }

    SmartDashboard.putBoolean("turning", turningFlag);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
