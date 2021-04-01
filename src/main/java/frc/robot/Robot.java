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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private BenSoloMotorSetup motorSetup = new BenSoloMotorSetup();

  private XboxController controller = new XboxController(0);

  private RadialDrive radialDrive = new RadialDrive(motorSetup);

  private AHRS navx = new AHRS();

  private PIDController turningController = new PIDController(0.0165, 0, 0.003);
  private PIDController driveController = new PIDController(0.025, 0, 0.0020);
  private PIDController radiusController = new PIDController(0.8, 0, 0.25);

  private PIDController smartDashboadController = turningController;

  private PathDriver path = new BenSoloPathDriver(motorSetup, navx, turningController, driveController,
      radiusController, radialDrive);

  private BenSoloRadialPathDriver benSoloRadialPathDriver = new BenSoloRadialPathDriver(motorSetup, navx,
      turningController, driveController, radiusController, radialDrive);

  private LimeLight limeLight = new LimeLight();

  SendableChooser<LinearSegment[]> sendableChooser = new SendableChooser<LinearSegment[]>();

  SendableChooser<Integer> galacticSearchSendableChooser = new SendableChooser<Integer>();

  public Robot() {

    turningController.setMin(-0.5);
    turningController.setMax(0.5);
    turningController.setTargetDetection(3, 15);

    driveController.setMin(-1);
    driveController.setMax(1);
    driveController.setTargetDetection(3, 15);

    radiusController.setMin(-0.9);
    radiusController.setMax(0.9);
    // driveController.setTargetDetection(5, 20);

    //path.loadSegments(GeneratedPath.MAIN);
    // benSoloRadialPathDriver.loadSegments(SavedPaths.MAIN2);

    sendableChooser.setDefaultOption("Barrel", SavedPaths.BARREL);
    sendableChooser.addOption("Slalom", SavedPaths.SLALOM);
    sendableChooser.addOption("Bounce", SavedPaths.BOUNCE);
    sendableChooser.addOption("A1", SavedPaths.A1);
    sendableChooser.addOption("A2", SavedPaths.A2);
    sendableChooser.addOption("B1", SavedPaths.B1);
    sendableChooser.addOption("B2", SavedPaths.B2);

    galacticSearchSendableChooser.setDefaultOption("AutoNav", 0);
    galacticSearchSendableChooser.addOption("Course A", 1);
    galacticSearchSendableChooser.addOption("Course B", 2);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    SmartDashboard.putData(sendableChooser);
    SmartDashboard.putData(galacticSearchSendableChooser);

    // startingAngle = navx.getAngle();
    // navx.setAngleAdjustment(-startingAngle);

    navx.zeroYaw();

    motorSetup.getLeftCanEncoder().setPosition(0);
    motorSetup.getRightCanEncoder().setPosition(0);

    SmartDashboard.putNumber("Kp", smartDashboadController.getKp());
    SmartDashboard.putNumber("Ki", smartDashboadController.getKi());
    SmartDashboard.putNumber("Kd", smartDashboadController.getKd());

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

    double targetOffsetAngleHorizontal = limeLight.getTargetOffsetAngleHorizontal();
    double targetOffsetAngleVertical = limeLight.getTargetOffsetAngleVertical();
    double targetArea = limeLight.getTargetArea();
    double targetSkew = limeLight.getTargetSkew();

    SmartDashboard.putNumber("_tx", targetOffsetAngleHorizontal);
    SmartDashboard.putNumber("_ty", targetOffsetAngleVertical);
    SmartDashboard.putNumber("_ta", targetArea);
    SmartDashboard.putNumber("_ts", targetSkew);

    System.out.println(String.format("x: %f, y: %f", targetOffsetAngleHorizontal, targetOffsetAngleVertical));

    double voltage = RobotController.getBatteryVoltage();

    SmartDashboard.putNumber("battery voltage", voltage);

    SmartDashboard.putNumber("left encoder", motorSetup.getLeftPositionInches());
    SmartDashboard.putNumber("right encoder", motorSetup.getRightPositionInches());
    SmartDashboard.putNumber("heading", navx.getAngle());

    double vLeft = motorSetup.getLeftVelocityInchesPerSecond();
    double vRight = motorSetup.getRightVelocityInchesPerSecond();

    SmartDashboard.putNumber("left velocity", vLeft);
    SmartDashboard.putNumber("right velocity", vRight);

    smartDashboadController.setKp(SmartDashboard.getNumber("Kp", 0));
    smartDashboadController.setKi(SmartDashboard.getNumber("Ki", 0));
    smartDashboadController.setKd(SmartDashboard.getNumber("Kd", 0));

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

    long t = System.nanoTime();
    SmartDashboard.putString("auto init time", t + "");


    if (galacticSearchSendableChooser.getSelected() == 0) { // Defers to manual path chooser

      path.loadSegments(sendableChooser.getSelected());

    } else if (galacticSearchSendableChooser.getSelected() == 1) { // Loads segments for Galactic Search course A

      if (limeLight.courseAFirstRedBallIsThere()) {
        path.loadSegments(SavedPaths.A1);
      } else {
        path.loadSegments(SavedPaths.A2);
      }

    } else { // Loads segments for Galactic Search course B

      if (limeLight.courseBFirstRedBallIsThere()) {
        path.loadSegments(SavedPaths.B1);
      } else {
        path.loadSegments(SavedPaths.B2);
      }

    }

    path.init();
    // benSoloRadialPathDriver.init();

  }

  boolean hasRun = false;

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // !true = false
    // !false = true

    /*
     * if hasRun is true, !hasRun evaluates to false, so the code inside the if
     * statement is not executed
     */

    if (!hasRun) {
      long t = System.nanoTime();
      SmartDashboard.putString("auto periodic time", t + "");

      hasRun = true;
    }

    path.periodic();
    // benSoloRadialPathDriver.periodic();

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // startingAngle = navx.getAngle();
    // navx.setAngleAdjustment(-startingAngle);
    navx.zeroYaw();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double targetOffsetAngleHorizontal = limeLight.getTargetOffsetAngleHorizontal();
    double targetOffsetAngleVertical = limeLight.getTargetOffsetAngleVertical();
    double targetArea = limeLight.getTargetArea();
    double targetSkew = limeLight.getTargetSkew();

    SmartDashboard.putNumber("_tx", targetOffsetAngleHorizontal);
    SmartDashboard.putNumber("_ty", targetOffsetAngleVertical);
    SmartDashboard.putNumber("_ta", targetArea);
    SmartDashboard.putNumber("_ts", targetSkew);

    double forwardAxis = controller.getRawAxis(3) - controller.getRawAxis(2);

    // forwardAxis = Utils.scaleAxis(forwardAxis);

    if (controller.getRawButton(1)) {
      forwardAxis = 0.5 * forwardAxis;
    }

    double turnAxis = controller.getRawAxis(0);
    double radius = Math.abs(Utils.getRadius(turnAxis));

    boolean left = turnAxis < 0;

    if (controller.getPOV() == 270) {
      radius = 24;
      left = true;

    } else if (controller.getPOV() == 90) {
      radius = 24;
      left = false;
    }

    if (controller.getRawButton(2)) {
      motorSetup.getLeftCanEncoder().setPosition(0);
      motorSetup.getRightCanEncoder().setPosition(0);
    }
    radialDrive.radialDrive(left, radius, forwardAxis);
    /*
     * if (controller.getRawButton(3)) { turningFlag = false; }
     * 
     * if (controller.getRawButton(6)) { turningFlag = true;
     * turningController.setTarget(navx.getAngle() + 90); } else
     * 
     * if (controller.getRawButton(5)) { turningFlag = true;
     * turningController.setTarget(navx.getAngle() - 90); } else
     * 
     * 
     * if (turningFlag) {
     * 
     * double speed = turningController.getControlOutput(navx.getAngle());
     * 
     * radialDrive.radialDrive(0, speed, false);
     * 
     * if (turningController.atTarget()) { turningFlag = false; }
     * 
     * } else { radialDrive.radialDrive(radius, forwardAxis); }
     * 
     * SmartDashboard.putBoolean("turning", turningFlag);
     */
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    radialDrive.radialDrive(false, 0, 0);

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    navx.zeroYaw();
    turningController.setTarget(90);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    double turnError = turningController.getTarget() - navx.getAngle();

    SmartDashboard.putNumber("turn error", turnError);

    double turnSpeed = turningController.getControlOutput(navx.getAngle());

    boolean left = turnSpeed < 0;

    SmartDashboard.putBoolean("turn left", left);
    SmartDashboard.putNumber("turn speed", turnSpeed);

    // TODO: fix negative radius
    radialDrive.radialDrive(left, 0, Math.abs(turnSpeed), false);

  }
}
