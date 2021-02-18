package frc.robot;

import java.util.ArrayList;
import java.util.List;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ContinuousPath {

    private List<Segment> segments = new ArrayList<>();
    private int step = 0;

    private RadialDrive drive;
    private PIDController turningController, driveController;
    private BenSoloMotorSetup motorSetup;
    private AHRS navx;
    private double lastSign = 0;

    public ContinuousPath(BenSoloMotorSetup motorSetup, AHRS navx, PIDController turningController,
            PIDController driveController, RadialDrive drive) {
        this.motorSetup = motorSetup;
        this.turningController = turningController;
        this.driveController = driveController;
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

            if(first.isTurning()) {

                lastSign = Math.signum(first.getTarget() - navx.getAngle());

            } else {

                lastSign = Math.signum(first.getTarget() - motorSetup.getLeftEncoderInches());

            }

        }

    }

    public void autoDrive() {

        // SmartDashboard.putString("turn info", String.format("step: %d, turning: %b",
        // step, turning));

        if (step >= segments.size()) {
            drive.radialDrive(0, 0);
            return;
        }

        Segment segment = segments.get(step);

        if (segment.isTurning()) {

            SmartDashboard.putNumber("turn error", turningController.getTarget() - navx.getAngle());

            drive.radialDrive(segment.getRadius(), 0.25, false);

            double currSign = Math.signum(segment.getTarget() - navx.getAngle());

            if (currSign != lastSign) {
                
                motorSetup.getLeftCanEncoder().setPosition(0);
                motorSetup.getRightCanEncoder().setPosition(0);

                step++;

                if (step < segments.size()) {

                    Segment next = segments.get(step);

                    if(next.isTurning()) {
        
                        lastSign = Math.signum(next.getTarget() - navx.getAngle());
        
                    } else {
        
                        lastSign = Math.signum(next.getTarget() - motorSetup.getLeftEncoderInches());
        
                    }

                }

            } else {
                lastSign = currSign;
            }

        

            

        } else {

            drive.radialDrive(RadialDrive.STRAIGHT_RADIUS, 0.25, false);

            double currSign = Math.signum(segment.getTarget() - motorSetup.getLeftEncoderInches());

            if (currSign != lastSign) {

                step++;

                if (step < segments.size()) {

                    Segment next = segments.get(step);

                    if(next.isTurning()) {
        
                        lastSign = Math.signum(next.getTarget() - navx.getAngle());
        
                    } else {
        
                        lastSign = Math.signum(next.getTarget() - motorSetup.getLeftEncoderInches());
        
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
