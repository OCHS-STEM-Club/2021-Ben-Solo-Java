package frc.robot;

import java.util.ArrayList;
import java.util.List;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Path {

    private List<Segment> segments = new ArrayList<>();
    private int step = 0;
    private boolean turning = true;

    private RadialDrive drive;
    private PIDController turningController, driveController, radiusController;
    private BenSoloMotorSetup motorSetup;
    private AHRS navx;

    public Path(BenSoloMotorSetup motorSetup, AHRS navx, PIDController turningController, PIDController driveController, PIDController radiusController,
            RadialDrive drive) {
        this.motorSetup = motorSetup;
        this.turningController = turningController;
        this.driveController = driveController;
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
        turning = true;
        motorSetup.getLeftCanEncoder().setPosition(0);
        motorSetup.getRightCanEncoder().setPosition(0);

        if (step < segments.size()) {
            turningController.setTarget(segments.get(step).getHeadingDegrees());
            driveController.setTarget(segments.get(step).getDistanceInches());
            radiusController.setTarget(segments.get(step).getHeadingDegrees());
        }

    }

    public void autoDrive() {

        SmartDashboard.putString("turn info", String.format("step: %d, turning: %b", step, turning));

        if (step >= segments.size()) {
            drive.radialDrive(false, 0, 0);
            return;
        }

        Segment segment = segments.get(step);

        SmartDashboard.putNumber("step", step);

        if (turning) {

            SmartDashboard.putNumber("turn error", turningController.getTarget() - navx.getAngle());

            boolean left = turningController.getTarget() - navx.getAngle() > 0;

            SmartDashboard.putBoolean("turn left", left);

            // TODO: fix negative radius
            drive.radialDrive(left, 0, turningController.getControlOutput(navx.getAngle()), false);

            if (turningController.atTarget()) {
                turning = false;
                motorSetup.getLeftCanEncoder().setPosition(0);
                motorSetup.getRightCanEncoder().setPosition(0);
            }

        } else {

            double output = radiusController.getControlOutput(navx.getAngle());

            SmartDashboard.putNumber("drive radius output", output);

            double radius = Math.abs(Utils.getRadius(output));

            drive.radialDrive(output < 0, radius, driveController.getControlOutput(motorSetup.getLeftPositionInches()), false);

            if (driveController.atTarget()) {

                step++;
                turning = true;

                if (step < segments.size()) {
                    turningController.setTarget(segments.get(step).getHeadingDegrees());
                    driveController.setTarget(segments.get(step).getDistanceInches());
                    radiusController.setTarget(segments.get(step).getHeadingDegrees());

                }

            }

        }

    }

    public static class Segment {

        private double headingDegrees, distanceInches;

        public Segment(double headingDegrees, double distanceInches) {
            this.headingDegrees = headingDegrees;
            this.distanceInches = distanceInches;
        }

        public double getDistanceInches() {
            return distanceInches;
        }

        public double getHeadingDegrees() {
            return headingDegrees;
        }

    }

}
