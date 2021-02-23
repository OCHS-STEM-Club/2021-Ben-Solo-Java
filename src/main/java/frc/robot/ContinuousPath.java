package frc.robot;

import java.util.ArrayList;
import java.util.List;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ContinuousPath {

    private List<Segment> segments = new ArrayList<>();
    private int step = 0;

    private RadialDrive drive;
    private PIDController turningController, driveController, radiusController;
    private BenSoloMotorSetup motorSetup;
    private AHRS navx;
    private double lastSign = 0;
    private double speed = 0.25;

    public ContinuousPath(BenSoloMotorSetup motorSetup, AHRS navx, PIDController turningController,
            PIDController driveController, PIDController radiusController, RadialDrive drive) {
        this.motorSetup = motorSetup;
        // this.turningController = turningController;
        // this.driveController = driveController;
        this.radiusController = radiusController;
        this.navx = navx;
        this.drive = drive;
    }

    public void addSegments(Segment... segments) {
        for (Segment segment : segments) {
            this.segments.add(segment);
        }
    }

    public void initDrive() {
        step = 0;
        motorSetup.getLeftCanEncoder().setPosition(0);
        motorSetup.getRightCanEncoder().setPosition(0);

        if (step < segments.size()) {
            /*
             * turningController.setTarget(segments.get(step).getHeadingDegrees());
             * driveController.setTarget(segments.get(step).getDistanceInches());
             */

            Segment first = segments.get(0);

            if (first.isTurning()) {

                lastSign = Math.signum(first.getTarget() - navx.getAngle());
                radiusController.setTarget(first.getTarget());

            } else {

                lastSign = Math.signum(first.getTarget() - motorSetup.getLeftEncoderInches());
                radiusController.setTarget(0);

            }

        }

    }

    public void autoDrive() {

        // SmartDashboard.putString("turn info", String.format("step: %d, turning: %b",
        // step, turning));

        SmartDashboard.putNumber("lastSign", lastSign);

        // stop at end
        if (step >= segments.size()) {
            drive.radialDrive(0, 0);
            return;
        }

        // get current segment
        Segment segment = segments.get(step);

        SmartDashboard.putNumber("target angle", radiusController.getTarget());

        if (segment.isTurning()) { // this segment is a turn

            // SmartDashboard.putNumber("turn error", turningController.getTarget() -
            // navx.getAngle());

            drive.radialDrive(segment.getRadius(), speed, false);

            double err = segment.getTarget() - navx.getAngle();

            double currSign = Math.signum(err);

            SmartDashboard.putNumber("currSign", currSign);
            SmartDashboard.putBoolean("sign change", currSign != lastSign);

            // String out = String.format("last: %d, curr: %d\n", (int)lastSign,
            // (int)currSign);

            // s.append(out);

            // System.out.print(out);

            // SmartDashboard.putString("sign log", s.toString());

            if (currSign != lastSign) {

                step++;

                if (step < segments.size()) {

                    Segment next = segments.get(step);

                    if (next.isTurning()) {

                        lastSign = Math.signum(next.getTarget() - navx.getAngle());
                        radiusController.setTarget(next.getTarget());

                    } else {

                        motorSetup.getLeftCanEncoder().setPosition(0);
                        motorSetup.getRightCanEncoder().setPosition(0);

                        lastSign = Math.signum(next.getTarget() - 0);

                    }

                }

            } else {

                lastSign = currSign;

            }

        } else { // this segment is not a turn

            double output = radiusController.getControlOutput(navx.getAngle());

            double radius = Utils.getRadius(output);

            drive.radialDrive(radius, speed, false);

            double err = segment.getTarget() - motorSetup.getLeftEncoderInches();

            double currSign = Math.signum(err);

            if (currSign != lastSign) {

                step++;

                if (step < segments.size()) {

                    Segment next = segments.get(step);

                    if (next.isTurning()) {

                        lastSign = Math.signum(next.getTarget() - navx.getAngle());
                        radiusController.setTarget(next.getTarget());

                    } else {

                        motorSetup.getLeftCanEncoder().setPosition(0);
                        motorSetup.getRightCanEncoder().setPosition(0);

                        lastSign = Math.signum(next.getTarget() - 0);

                    }

                }

            } else {

                lastSign = currSign;

            }

        }

    }

    public static class Segment {

        private double target;

        private boolean turning;

        private double radius;

        private Segment(boolean turning, double target, double radius) {
            this.target = target;
            this.turning = turning;
            this.radius = radius;
        }

        public static Segment turn(double endHeading, double radius) {
            return new Segment(true, endHeading, radius);
        }

        public static Segment straight(double distance) {
            return new Segment(false, distance, RadialDrive.STRAIGHT_RADIUS);
        }

        public double getTarget() {
            return target;
        }

        public boolean isTurning() {
            return turning;
        }

        public double getRadius() {
            return radius;
        }

    }

}
