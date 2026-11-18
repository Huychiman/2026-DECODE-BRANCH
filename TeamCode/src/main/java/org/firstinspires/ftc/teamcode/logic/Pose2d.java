package org.firstinspires.ftc.teamcode.logic;

/**
 * Immutable robot pose on the field.
 *
 * <p>Uses the FTC SDK field frame (same frame as {@code AprilTagGameDatabase.getDecodeTagLibrary()}):
 * origin at field centre, inches, the GOAL wall at -X, RED half at +Y, BLUE half at -Y. Heading is
 * in radians, counter-clockwise positive, 0 = facing +X (towards the audience wall).
 */
public final class Pose2d {
    public final double x;
    public final double y;
    public final double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public static Pose2d degrees(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    public double distanceTo(Pose2d o) {
        return Math.hypot(o.x - x, o.y - y);
    }

    public double distanceTo(double ox, double oy) {
        return Math.hypot(ox - x, oy - y);
    }

    /** Field heading (radians) that points from this pose towards (tx, ty). */
    public double headingTo(double tx, double ty) {
        return Math.atan2(ty - y, tx - x);
    }

    public Pose2d withHeading(double h) {
        return new Pose2d(x, y, h);
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "(%.1f, %.1f, %.1f°)", x, y, Math.toDegrees(heading));
    }
}
