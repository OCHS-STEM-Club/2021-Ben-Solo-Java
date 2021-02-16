package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;

// hello world
public class RadialDrive {

    private static final double ROBOT_WIDTH = 20.5;

    public static final double STRAIGHT_RADIUS = 1e8;

    public static final double SPEED_LIMIT = 0.6;

    private double RADIUS_SCALE_FACTOR = 1;

    private SpeedController leftGroup, rightGroup;

    public RadialDrive(SpeedController leftGroup, SpeedController rightGroup) {
        this.leftGroup = leftGroup;
        this.rightGroup = rightGroup;
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

        if (limit) {
            leftSpeedFactor *= SPEED_LIMIT;
            rightSpeedFactor *= SPEED_LIMIT;
        }

        leftGroup.set(-leftSpeedFactor);
        rightGroup.set(rightSpeedFactor);

    }

}
