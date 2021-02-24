package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// hello world
public class RadialDrive {

    private static final double ROBOT_WIDTH = 20.5;

    public static final double STRAIGHT_RADIUS = 1e8;

    public static final double SPEED_LIMIT = 0.6;

    private double RADIUS_SCALE_FACTOR = 1;

    private SpeedController leftGroup, rightGroup;
    private BenSoloMotorSetup motorSetup;

    private PIDController relativeSpeedController = new PIDController(0, 0, 0);

    private RollingAverage ravg = new RollingAverage(10);
    private RollingAverage lavg = new RollingAverage(10);
    private RollingAverage avgRatio = new RollingAverage(25);


    public PIDController getRelativeSpeedController() {
        return relativeSpeedController;
    }

    public RadialDrive(BenSoloMotorSetup motorSetup) {
        this.motorSetup = motorSetup;
        this.leftGroup = motorSetup.getLeftMotorController();
        this.rightGroup = motorSetup.getRightMotorController();

        relativeSpeedController.setMax(1);
        relativeSpeedController.setMin(-1);
    }

    public void radialDrive(double radius, double forwardAxis) {
        radialDrive(radius, forwardAxis, true);
    }

    public void radialDrive(double radius, double forwardAxis, boolean limit) {

        radius *= RADIUS_SCALE_FACTOR;

        double absRaidus = Math.abs(radius);

        double turnScale = forwardAxis * (ROBOT_WIDTH / 2) / ((ROBOT_WIDTH / 2) + absRaidus);
        double forwardScale = -forwardAxis * (-(ROBOT_WIDTH / 2) / ((ROBOT_WIDTH / 2) + absRaidus) + 1);

        double leftSpeedFactor = -Utils.sign(radius) * (turnScale) + forwardScale;
        double rightSpeedFactor = Utils.sign(radius) * (turnScale) + forwardScale;

        double targetRatio = leftSpeedFactor/rightSpeedFactor;

        double currentRatio = motorSetup.getLeftCanEncoder().getVelocity() / -motorSetup.getRightCanEncoder().getVelocity();

        lavg.feed(motorSetup.getLeftCanEncoder().getVelocity());
        ravg.feed(motorSetup.getRightCanEncoder().getVelocity());

        avgRatio.feed(currentRatio);

        SmartDashboard.putNumber("target ratio", targetRatio);
        SmartDashboard.putNumber("current ratio", currentRatio);

        relativeSpeedController.setTarget(targetRatio);

        double output = relativeSpeedController.getControlOutput(avgRatio.avg());

        if(Double.isNaN(targetRatio) || Math.abs(radius) > 250) {
            output = rightSpeedFactor;
        }

        if (limit) {
            leftSpeedFactor *= SPEED_LIMIT;
            rightSpeedFactor *= SPEED_LIMIT;
            output *= SPEED_LIMIT;
        }

        

        //SmartDashboard.putNumber("left avg", lavg.avg());
        //SmartDashboard.putNumber("right avg", ravg.avg());
        SmartDashboard.putNumber("avg ratio", avgRatio.avg());

        SmartDashboard.putNumber("radius", radius);


        SmartDashboard.putNumber("left speed", leftSpeedFactor);
        SmartDashboard.putNumber("right speed", rightSpeedFactor);
        SmartDashboard.putNumber("output speed", output);

        leftGroup.set(-leftSpeedFactor);
        rightGroup.set(rightSpeedFactor);
        //rightGroup.set(output);

    }

}
