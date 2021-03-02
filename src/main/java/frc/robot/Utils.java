package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class Utils {

    private static double INPUT_SCALE_FACTOR = 500;
    private static double NEGATIVE_ZERO = -0.0d;

    public static double scaleAxis(double getRawAxis) {

        return Math.signum(getRawAxis) * Math.pow(Math.abs(getRawAxis), 1.5);

    }

    public static double getRadius(double turnProp) {

        if (turnProp == 0) {
            return 1e12;
        }

        return INPUT_SCALE_FACTOR * (1 - Math.abs(turnProp)) / turnProp;

    }

    public static int sign(double value) {
        if (Double.doubleToRawLongBits(value) == Double.doubleToRawLongBits(NEGATIVE_ZERO)) {
            return -1;
        } else if (value == 0) {
            return 1;
        } else if (value > 0) {
            return 1;
        } else {
            return -1;
        }
    }

    public static double modulo(double in, double by) {

        double ret = in % by;

        if (ret < 0) {
            ret += by;
        }

        return ret;

    }

    public static double boundedAngle(AHRS ahrs) {
        return modulo(ahrs.getAngle(), 360);
    }

    public static boolean inRange(double value, double min, double max) {

        return value <= max && value >= min;

    }

    public static double minAngleDiff(double angle, double target) {

        double a1 = angle - target;
        double a2 = 360 - angle - target;

        if (Math.abs(a1) < Math.abs(a2)) {
            return (a1);
        } else {
            return (a2);
        }

    }

}
