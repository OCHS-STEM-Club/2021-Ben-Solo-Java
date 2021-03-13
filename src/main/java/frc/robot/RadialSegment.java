package frc.robot;

public class RadialSegment {

    private double heading, distance, radius;
    private boolean left;

    private RadialSegment(double heading, double distance, double radius, boolean left) {
        this.heading = heading;
        this.distance = distance;
        this.radius = radius;
        this.left = left;
    }

    public static RadialSegment straight(double heading, double distance) {
        return new RadialSegment(heading, distance, -1, false);
    }

    public static RadialSegment turning(double heading, double radius, boolean left) {
        return new RadialSegment(heading, 0, radius, left);

    }

    public boolean isTurn() {
        return radius >= 0;
    }

    public boolean isStraight() {
        return radius < 0;
    }

    public double getHeadingDegrees() {
        return heading;
    }

    public double getDistanceInches() {
        return distance;
    }

    public double getRadius() {
        return radius;
    }

    public boolean isTurningLeft() {
        return left;
    }
}
