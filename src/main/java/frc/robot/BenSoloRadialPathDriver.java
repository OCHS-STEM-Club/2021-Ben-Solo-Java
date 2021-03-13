package frc.robot;


import com.kauailabs.navx.frc.AHRS;


public class BenSoloRadialPathDriver extends RadialPathDriver {

    private RadialDrive drive;
    private PIDController turningController, driveController, turnAmountController;
    private BenSoloMotorSetup motorSetup;
    private AHRS navx;
    private double lastSign = 0;
    private double speed = 0.17;

    public BenSoloRadialPathDriver(BenSoloMotorSetup motorSetup, AHRS navx, PIDController turningController,
            PIDController driveController, PIDController radiusController, RadialDrive drive) {
        this.motorSetup = motorSetup;
        // this.turningController = turningController;
        // this.driveController = driveController;
        this.turnAmountController = radiusController;
        this.navx = navx;
        this.drive = drive;
    }

    @Override
    protected void pathInit() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void arcInit(double headingDegrees, double radius, boolean left, int i) {
        // TODO Auto-generated method stub

    }

    /**
     * @return true if turn is complete, false if turn is still in progress
     * 
     */
    @Override
    protected boolean arcPeriodic(double headingDegrees, double radius, boolean left, int i) {
        drive.radialDrive(left, radius, speed, true); // drives the robot 
        //navx.
        return false;
    }

    @Override
    protected void straightInit(double headingDegrees, double distanceInches, int i) {
        // TODO Auto-generated method stub

    }

    @Override
    protected boolean straightPeriodic(double headingDegrees, double distanceInches, int i) {
        double output = turnAmountController.getControlOutput(navx.getAngle());
        double getRadius = Math.abs(Utils.getRadius(output));
        drive.radialDrive(output < 0, getRadius, speed, true);
        return false;
    }

    @Override
    protected void pathStop() {
        drive.radialDrive(false, 0, 0);

        

    }

   

}
