// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

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

  private Path path = new Path(radialDrive);

  private AHRS navx = new AHRS();

  private double startingAngle = 0;

  private boolean turningFlag = false;

  private double turnTargetAngle = 0;

  private PIDController turningController = new PIDController(0.01, 0, 0.0011);
  private PIDController driveController = new PIDController(0.019, 0, 0);

  public Robot() {
    path.addSegments(GeneratedPath.MAIN);
    turningController.setMin(-0.6);
    turningController.setMax(0.6);
    driveController.setMin(-0.6);
    driveController.setMax(0.6);

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    startingAngle = navx.getAngle();

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
    motorSetup.getLeftCanEncoder().setPosition(0);
    motorSetup.getRightCanEncoder().setPosition(0);

    // path.initDrive();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // path.autoDrive();

    SmartDashboard.putNumber("left encoder", motorSetup.getLeftEncoderInches());
    SmartDashboard.putNumber("right encoder", motorSetup.getRightEncoderInches());

    double speed = driveController.getValue(120 - motorSetup.getLeftEncoderInches());

    radialDrive.radialDrive(RadialDrive.STRAIGHT_RADIUS, speed);

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    startingAngle = navx.getAngle();
    turningFlag = false;

    SmartDashboard.putNumber("Kp", 0.01);
    SmartDashboard.putNumber("Ki", 0);
    SmartDashboard.putNumber("Kd", 0.0011);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    turningController.setKp(SmartDashboard.getNumber("Kp", 0));
    turningController.setKi(SmartDashboard.getNumber("Ki", 0));
    turningController.setKd(SmartDashboard.getNumber("Kd", 0));

    double forwardAxis = controller.getRawAxis(3) - controller.getRawAxis(2);

    forwardAxis = Utils.scaleAxis(forwardAxis);

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
      turnTargetAngle = navx.getAngle() + 90;
    } else

    if (controller.getRawButton(5)) {
      turningFlag = true;
      turnTargetAngle = navx.getAngle() - 90;
    } else

    if (turningFlag) {

      double speed = turningController.getValue(turnTargetAngle - navx.getAngle());

      // speed = Math.max(0.2, speed);

      SmartDashboard.putNumber("speed", speed);

      radialDrive.radialDrive(0, speed, false);

      if (turningController.atTarget()) {
        turningFlag = false;
      }

    } else {
      radialDrive.radialDrive(radius, forwardAxis);
    }

    SmartDashboard.putNumber("left encoder", motorSetup.getLeftEncoderInches());
    SmartDashboard.putNumber("right encoder", motorSetup.getRightEncoderInches());
    SmartDashboard.putNumber("heading", navx.getAngle() - startingAngle);
    SmartDashboard.putBoolean("turning", turningFlag);

    SmartDashboard.putNumber("encoder factor", motorSetup.getRightCanEncoder().getPositionConversionFactor());

    if (controller.getRawButton(4)) {
      radialDrive.radialDrive(RadialDrive.STRAIGHT_RADIUS, 0.075);
    }

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
