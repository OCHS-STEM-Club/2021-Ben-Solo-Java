package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BenSoloRadialPathDriver extends RadialPathDriver {

    private RadialDrive drive;
    private PIDController turningController, driveController, correctionAmountController;
    private BenSoloMotorSetup motorSetup;
    private AHRS navx;
    private double lastSign = 0;
    private double speed = 0.17;

    public BenSoloRadialPathDriver(BenSoloMotorSetup motorSetup, AHRS navx, PIDController turningController,
            PIDController driveController, PIDController radiusController, RadialDrive drive) {
        this.motorSetup = motorSetup;
        // this.turningController = turningController;
        // this.driveController = driveController;
        this.correctionAmountController = radiusController;
        this.navx = navx;
        this.drive = drive;
    }

    @Override
    protected void pathInit() {
        navx.zeroYaw();

    }

    @Override
    protected void arcInit(double headingDegrees, double radius, boolean left, int i) {

    }

    /**
     * @return true if turn is complete, false if turn is still in progress
     * 
     */
    @Override
    protected boolean arcPeriodic(double headingDegrees, double radius, boolean left, int i) {
        drive.radialDrive(left, radius, speed, false); // drives the robot
        double currentHeadingDegrees = navx.getAngle();

        /*
         * if robot is turing left, the headingDegrees is decreasing, if headingDegrees
         * has become less than target then return true if robot is turning right, the
         * headingDegrees is increasing, if headingDegrees has become greater than
         * target then return true
         */
        if (left) 
            return currentHeadingDegrees <= headingDegrees;
        else
            return currentHeadingDegrees >= headingDegrees;
    }

    @Override
    protected void straightInit(double headingDegrees, double distanceInches, int i) {
        motorSetup.getLeftCanEncoder().setPosition(0);// resets encoders to prepare for next segment
        motorSetup.getRightCanEncoder().setPosition(0);
        correctionAmountController.setTarget(headingDegrees);// setting the target heading for the PID controller

    }

    /**
     * @return true is straight segment is complete, false if straight segment is
     *         still in progress
     * 
     */

    @Override
    protected boolean straightPeriodic(double headingDegrees, double targetDistanceInches, int i) {
        double correctionAmount = correctionAmountController.getControlOutput(navx.getAngle());
        double getRadius = Math.abs(Utils.getRadius(correctionAmount));
        double currentDistanceInches = motorSetup.getLeftPositionInches(); // how far the robot has moved
        drive.radialDrive(correctionAmount < 0, getRadius, speed, false);

        SmartDashboard.putNumber("target distance", targetDistanceInches);

        // if robot has gone desired distance or farther, returns true; if not, returns
        // false and keeps going
        return currentDistanceInches >= targetDistanceInches;

    }

    @Override
    protected void pathStop() {
        drive.radialDrive(false, 0, 0);

    }

}
