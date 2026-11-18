package org.firstinspires.ftc.teamcode.logic;

/**
 * Mecanum maths with the convention: forward = +, strafe LEFT = +, turn COUNTER-CLOCKWISE = +.
 *
 * <p>Wheel powers are "positive = that wheel drives the robot forward". Motor direction flags in
 * {@link org.firstinspires.ftc.teamcode.config.RobotConfig} make that true on the real robot.
 */
public final class MecanumKinematics {
    private MecanumKinematics() {}

    /** Wheel powers in the order front-left, front-right, back-left, back-right. */
    public static final class WheelPowers {
        public final double fl, fr, bl, br;

        public WheelPowers(double fl, double fr, double bl, double br) {
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
        }

        public double maxAbs() {
            return Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        }

        public WheelPowers scaled(double s) {
            return new WheelPowers(fl * s, fr * s, bl * s, br * s);
        }

        public double[] toArray() {
            return new double[] {fl, fr, bl, br};
        }
    }

    /**
     * Robot-centric mecanum mix. Output is normalised so no wheel exceeds 1 while the ratio between
     * axes is preserved (same normalisation the NextFTC driver command uses).
     */
    public static WheelPowers robotCentric(double forward, double left, double turnCcw) {
        double fl = forward - left - turnCcw;
        double fr = forward + left + turnCcw;
        double bl = forward + left - turnCcw;
        double br = forward - left + turnCcw;
        double denom = Math.max(Math.abs(forward) + Math.abs(left) + Math.abs(turnCcw), 1.0);
        return new WheelPowers(fl / denom, fr / denom, bl / denom, br / denom);
    }

    /**
     * Rotates a field-frame translation request into the robot frame.
     *
     * @return {forward, left}
     */
    public static double[] fieldToRobot(double fieldX, double fieldY, double robotHeading) {
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);
        double forward = fieldX * cos + fieldY * sin;
        double left = -fieldX * sin + fieldY * cos;
        return new double[] {forward, left};
    }

    /** Inverse of {@link #fieldToRobot}. @return {fieldX, fieldY} */
    public static double[] robotToField(double forward, double left, double robotHeading) {
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);
        return new double[] {forward * cos - left * sin, forward * sin + left * cos};
    }

    /**
     * Forward kinematics: wheel travel (inches, positive = forward) to robot-frame displacement.
     *
     * @return {forward, left} in inches
     */
    public static double[] wheelDeltasToRobot(double dfl, double dfr, double dbl, double dbr) {
        double forward = (dfl + dfr + dbl + dbr) / 4.0;
        double left = (-dfl + dfr + dbl - dbr) / 4.0;
        return new double[] {forward, left};
    }
}
