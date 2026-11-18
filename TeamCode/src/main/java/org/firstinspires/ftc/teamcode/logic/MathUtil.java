package org.firstinspires.ftc.teamcode.logic;

/** Small math helpers shared by the robot logic. Pure Java so it can be unit tested on a PC. */
public final class MathUtil {
    private MathUtil() {}

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Wraps an angle in radians to the range (-PI, PI]. */
    public static double wrapAngle(double radians) {
        double a = radians % (2.0 * Math.PI);
        if (a <= -Math.PI) a += 2.0 * Math.PI;
        if (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    /** Zeroes small joystick noise and rescales the rest so output still reaches +/-1. */
    public static double deadband(double v, double band) {
        if (Math.abs(v) <= band) return 0.0;
        return Math.signum(v) * (Math.abs(v) - band) / (1.0 - band);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}
