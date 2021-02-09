package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class Path {

    private List<Segment> segments = new ArrayList<>();
    private int step = 0;
    private RadialDrive drive;
    private long t1;
    private long t2;

    public static class Builder {

        private List<Segment> segments = new ArrayList<>();

        public Builder add(double radius, double forwardAxis, long time) {
            segments.add(new Segment(radius, forwardAxis, time));
            return this;
        }

        public Path.Segment[] build() {
            return segments.toArray(new Segment[0]);
        }

    }

    public Path(RadialDrive drive) {
        this.drive = drive;

    }

    public void addSegments(Segment... segments) {
        for (Segment segment : segments) {
            this.segments.add(segment);
        }

        this.segments.add(new Segment(RadialDrive.STRAIGHT_RADIUS, -.25, 100));
    }

    public void initDrive() {
        step= 0;
        t1 = System.currentTimeMillis();

    }

    public void autoDrive() {

        if (step >= segments.size()) {
            drive.radialDrive(0, 0, false);
            return;
        }

        t2 = System.currentTimeMillis();
        Segment segment = segments.get(step);
        drive.radialDrive(segment.getRadius(), segment.getForwardAxis(), false);

        if (t2 - t1 > segment.getTime()) {
            step++;
            t1 = System.currentTimeMillis();
        }

    }

    public static class Segment {
        private double radius, forwardAxis;
        private long time;

        public Segment(double radius, double forwardAxis, long time) {
            this.radius = radius;
            this.forwardAxis = forwardAxis;
            this.time = time;

        }

        public double getRadius() {
            return radius;
        }

        public double getForwardAxis() {
            return forwardAxis;
        }

        public long getTime() {
            return time;

        }

    }

}
