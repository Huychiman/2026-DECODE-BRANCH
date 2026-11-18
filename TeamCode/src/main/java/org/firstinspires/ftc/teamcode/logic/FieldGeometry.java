package org.firstinspires.ftc.teamcode.logic;

/**
 * DECODE field geometry in the FTC SDK field frame (inches, origin at centre).
 *
 * <p>Sources: goal tag positions are copied from the SDK 11.0 {@code AprilTagGameDatabase} DECODE
 * library (BlueTarget id 20 at (-58.37, -55.64, 29.5), RedTarget id 24 at (-58.37, +55.64, 29.5)).
 * Launch zones follow the Competition Manual section 9.3: the GOAL-side zone is a triangle 6 tiles
 * wide by 3 tiles deep (both goal-side corners to field centre) and the audience-side zone a triangle
 * 2 tiles wide by 1 tile deep. The launch-line tape counts as inside the zone.
 */
public final class FieldGeometry {
    private FieldGeometry() {}

    public static final double HALF_FIELD = 72.0;
    public static final double TILE = 24.0;

    public static final double GOAL_X = -58.3727;
    public static final double GOAL_Y_RED = 55.6425;
    public static final double GOAL_TAG_HEIGHT = 29.5;

    /** Goal-side launch zone triangle (x <= -|y|). */
    public static final double[][] GOAL_ZONE = {{-72, 72}, {-72, -72}, {0, 0}};
    /** Audience-side launch zone triangle. */
    public static final double[][] AUDIENCE_ZONE = {{72, 24}, {72, -24}, {48, 0}};

    public static boolean pointInTriangle(double px, double py, double[][] t) {
        double d1 = cross(px, py, t[0], t[1]);
        double d2 = cross(px, py, t[1], t[2]);
        double d3 = cross(px, py, t[2], t[0]);
        boolean hasNeg = d1 < -1e-9 || d2 < -1e-9 || d3 < -1e-9;
        boolean hasPos = d1 > 1e-9 || d2 > 1e-9 || d3 > 1e-9;
        return !(hasNeg && hasPos);
    }

    private static double cross(double px, double py, double[] a, double[] b) {
        return (px - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (py - b[1]);
    }

    /** The four corners of a square robot footprint. */
    public static double[][] robotCorners(Pose2d pose, double robotLength, double robotWidth) {
        double hl = robotLength / 2.0, hw = robotWidth / 2.0;
        double[][] local = {{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}};
        double[][] out = new double[4][2];
        for (int i = 0; i < 4; i++) {
            double[] f = MecanumKinematics.robotToField(local[i][0], local[i][1], pose.heading);
            out[i][0] = pose.x + f[0];
            out[i][1] = pose.y + f[1];
        }
        return out;
    }

    private static boolean footprintTouches(double[][] zone, Pose2d pose, double len, double wid) {
        // Sample the footprint densely: exact polygon intersection is overkill for a 2-triangle field.
        double[][] c = robotCorners(pose, len, wid);
        for (int i = 0; i <= 10; i++) {
            for (int j = 0; j <= 10; j++) {
                double u = i / 10.0, v = j / 10.0;
                double ax = MathUtil.lerp(c[0][0], c[1][0], u), ay = MathUtil.lerp(c[0][1], c[1][1], u);
                double bx = MathUtil.lerp(c[3][0], c[2][0], u), by = MathUtil.lerp(c[3][1], c[2][1], u);
                if (pointInTriangle(MathUtil.lerp(ax, bx, v), MathUtil.lerp(ay, by, v), zone)) return true;
            }
        }
        return false;
    }

    /** True when any part of the robot is inside or over a launch zone, i.e. it may legally launch. */
    public static boolean canLaunchFrom(Pose2d pose, double len, double wid) {
        return footprintTouches(GOAL_ZONE, pose, len, wid) || footprintTouches(AUDIENCE_ZONE, pose, len, wid);
    }

    /** LEAVE: robot no longer over any launch line, i.e. entirely outside both zones. */
    public static boolean hasLeft(Pose2d pose, double len, double wid) {
        return !canLaunchFrom(pose, len, wid);
    }

    public static boolean insideField(Pose2d pose, double len, double wid) {
        for (double[] p : robotCorners(pose, len, wid)) {
            if (Math.abs(p[0]) > HALF_FIELD + 1e-6 || Math.abs(p[1]) > HALF_FIELD + 1e-6) return false;
        }
        return true;
    }

    /**
     * Converts a camera-to-tag straight-line range into floor distance (the ShotTable key), given the
     * camera lens height. Returns NaN for impossible inputs.
     */
    public static double floorDistanceFromRange(double rangeInches, double cameraHeightInches) {
        if (!MathUtil.isFinite(rangeInches) || rangeInches <= 0) return Double.NaN;
        double dz = GOAL_TAG_HEIGHT - cameraHeightInches;
        double sq = rangeInches * rangeInches - dz * dz;
        return sq > 0 ? Math.sqrt(sq) : 0.0;
    }

    /** Floor distance from robot centre to our goal's AprilTag: the ShotTable key. */
    public static double distanceToGoal(Pose2d pose, Alliance alliance) {
        return pose.distanceTo(alliance.goalX(), alliance.goalY());
    }
}
