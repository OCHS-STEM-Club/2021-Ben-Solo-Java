package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// hello world
public class RadialDrive {

    private static final double ROBOT_WIDTH = 17.5;

    public static final double STRAIGHT_RADIUS = 1e8;

    public static final double SPEED_LIMIT = 0.6;

    private SpeedController leftGroup, rightGroup;
    private BenSoloMotorSetup motorSetup;

    private PIDController leftInchesPerSecondController = new PIDController(0.2, 0, 0.01);
    private PIDController rightInchesPerSecondController = new PIDController(0.2, 0, 0.01);

    RollingAverage la = new RollingAverage(10);
    RollingAverage ra = new RollingAverage(10);

    double loff, roff;


    public RadialDrive(BenSoloMotorSetup motorSetup) {
        this.motorSetup = motorSetup;
        this.leftGroup = motorSetup.getLeftMotorController();
        this.rightGroup = motorSetup.getRightMotorController();

        leftInchesPerSecondController.setMin(-10);
        leftInchesPerSecondController.setMax(10);
        rightInchesPerSecondController.setMin(-10);
        rightInchesPerSecondController.setMax(10);

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
        double max = Math.max(Math.abs(leftProp), Math.abs(rightProp));
        leftProp *= forwardAxis / max;
        rightProp *= forwardAxis / max;

        if (limit) {
            leftProp *= SPEED_LIMIT;
            rightProp *= SPEED_LIMIT;
        }

        if (left) {
            // swapping the left and right prop values to turn left
            double tempLeftProp = leftProp;
            leftProp = rightProp;
            rightProp = tempLeftProp;
        }

        leftGroup.set(leftProp);
        rightGroup.set(-rightProp);

    }

    public void setIPSTarget(boolean left, double radius, double inchesPerSecond) {
        if (radius < 0) {
            System.out.println("Radius cannot be negative: " + radius);
        }

        radius = Math.abs(radius);

        double leftPos = -ROBOT_WIDTH / 2;
        double rightPos = ROBOT_WIDTH / 2;
        double leftProp = radius - leftPos;
        double rightProp = radius - rightPos;
        double max = Math.max(Math.abs(leftProp), Math.abs(rightProp));
        leftProp /= max;
        rightProp /= max;

        if (left) {
            // swapping the left and right prop values to turn left
            double tempLeftProp = leftProp;
            leftProp = rightProp;
            rightProp = tempLeftProp;
        }

        double leftInchesPerSecond = leftProp * inchesPerSecond;
        double rightInchesPerSecond = rightProp * inchesPerSecond;

        loff = leftInchesPerSecond;
        roff = rightInchesPerSecond;

        leftInchesPerSecondController.setTarget(leftInchesPerSecond);
        rightInchesPerSecondController.setTarget(rightInchesPerSecond);
    }

    public void radialDriveInchesPerSecond(boolean left, double radius, double inchesPerSecond) {
        if (radius < 0) {
            System.out.println("Radius cannot be negative: " + radius);
        }

        radius = Math.abs(radius);

        double leftPos = -ROBOT_WIDTH / 2;
        double rightPos = ROBOT_WIDTH / 2;
        double leftProp = radius - leftPos;
        double rightProp = radius - rightPos;
        double max = Math.max(Math.abs(leftProp), Math.abs(rightProp));
        leftProp /= max;
        rightProp /= max;

        if (left) {
            // swapping the left and right prop values to turn left
            double tempLeftProp = leftProp;
            leftProp = rightProp;
            rightProp = tempLeftProp;
        }

        double leftInchesPerSecond = leftProp * inchesPerSecond;
        double rightInchesPerSecond = rightProp * inchesPerSecond;

        leftInchesPerSecondController.setTarget(leftInchesPerSecond);
        rightInchesPerSecondController.setTarget(rightInchesPerSecond);

        double currLeftIPS = motorSetup.getLeftVelocityInchesPerSecond();
        double currRightIPS = -motorSetup.getRightVelocityInchesPerSecond();

        la.feed(currLeftIPS);
        ra.feed(currRightIPS);

        double leftOutput = leftInchesPerSecondController.getControlOutput(la.avg());
        double rightOutput = rightInchesPerSecondController.getControlOutput(ra.avg());

        SmartDashboard.putNumber("leftOut", leftOutput);
        SmartDashboard.putNumber("rightOut", rightOutput);

        SmartDashboard.putString("leftIPS", String.format("target: %f, current: %f", leftInchesPerSecond, currLeftIPS));
        SmartDashboard.putString("rightIPS", String.format("target: %f, current: %f", rightInchesPerSecond, currRightIPS));

        double vLeft = inchesPerSecond/17.6 * leftProp + leftOutput;
        double vRight = inchesPerSecond/17.6 * rightProp + rightOutput;

        SmartDashboard.putNumber("left voltage", vLeft);
        SmartDashboard.putNumber("right voltage", vRight);

        // 0V = 0 in/s
        // 1V = 13.5 in/s
        // 1.8V = 30 in/s
        // 2.5V = 44 in/s
        // 3.6V = 66.5 in/s
        // 5V = 95 in/s

        // in/s = 17.6(V)
        leftGroup.setVoltage(vLeft); 
        rightGroup.setVoltage(-vRight); 
    }

    public void radialDriveTarget() {
        
        double currLeftIPS = motorSetup.getLeftVelocityInchesPerSecond();
        double currRightIPS = -motorSetup.getRightVelocityInchesPerSecond();

        la.feed(currLeftIPS);
        ra.feed(currRightIPS);

        double leftOutput = leftInchesPerSecondController.getControlOutput(la.avg());
        double rightOutput = rightInchesPerSecondController.getControlOutput(ra.avg());

        SmartDashboard.putNumber("leftOut", leftOutput);
        SmartDashboard.putNumber("rightOut", rightOutput);

        //SmartDashboard.putString("leftIPS", String.format("target: %f, current: %f", leftInchesPerSecond, currLeftIPS));
        //SmartDashboard.putString("rightIPS", String.format("target: %f, current: %f", rightInchesPerSecond, currRightIPS));

        double vLeft = loff/17.6 + leftOutput;
        double vRight = roff/17.6 + rightOutput;

        SmartDashboard.putNumber("left voltage", vLeft);
        SmartDashboard.putNumber("right voltage", vRight);

        // 0V = 0 in/s
        // 1V = 13.5 in/s
        // 1.8V = 30 in/s
        // 2.5V = 44 in/s
        // 3.6V = 66.5 in/s
        // 5V = 95 in/s

        // in/s = 17.6(V)
        leftGroup.setVoltage(vLeft); 
        rightGroup.setVoltage(-vRight); 
    }

    /*
     * public void radialDrive(double radius, double forwardAxis, boolean limit) {
     * radialDrive(false, radius, forwardAxis, limit); }
     */
    /*
     * public void radialDrive(double radius, double forwardAxis, boolean limit) {
     * 
     * radius *= RADIUS_SCALE_FACTOR;
     * 
     * double absRaidus = Math.abs(radius);
     * 
     * double turnScale = forwardAxis * (ROBOT_WIDTH / 2) / ((ROBOT_WIDTH / 2) +
     * absRaidus); double forwardScale = -forwardAxis * (-(ROBOT_WIDTH / 2) /
     * ((ROBOT_WIDTH / 2) + absRaidus) + 1);
     * 
     * double leftSpeedFactor = -Utils.sign(radius) * (turnScale) + forwardScale;
     * double rightSpeedFactor = Utils.sign(radius) * (turnScale) + forwardScale;
     * 
     * double targetRatio = leftSpeedFactor / rightSpeedFactor;
     * 
     * double currentRatio = motorSetup.getLeftCanEncoder().getVelocity() /
     * -motorSetup.getRightCanEncoder().getVelocity();
     * 
     * lavg.feed(motorSetup.getLeftCanEncoder().getVelocity());
     * ravg.feed(motorSetup.getRightCanEncoder().getVelocity());
     * 
     * avgRatio.feed(currentRatio);
     * 
     * SmartDashboard.putNumber("target ratio", targetRatio);
     * SmartDashboard.putNumber("current ratio", currentRatio);
     * 
     * relativeSpeedController.setTarget(targetRatio);
     * 
     * double output = relativeSpeedController.getControlOutput(avgRatio.avg());
     * 
     * if (Double.isNaN(targetRatio) || Math.abs(radius) > 250) { output =
     * rightSpeedFactor; }
     * 
     * if (limit) { leftSpeedFactor *= SPEED_LIMIT; rightSpeedFactor *= SPEED_LIMIT;
     * output *= SPEED_LIMIT; }
     * 
     * // SmartDashboard.putNumber("left avg", lavg.avg()); //
     * SmartDashboard.putNumber("right avg", ravg.avg());
     * SmartDashboard.putNumber("avg ratio", avgRatio.avg());
     * 
     * SmartDashboard.putNumber("radius", radius);
     * 
     * SmartDashboard.putNumber("left speed", leftSpeedFactor);
     * SmartDashboard.putNumber("right speed", rightSpeedFactor);
     * SmartDashboard.putNumber("output speed", output);
     * 
     * leftGroup.set(-leftSpeedFactor); rightGroup.set(rightSpeedFactor); //
     * rightGroup.set(output);
     * 
     * }
     * 
     */

}
