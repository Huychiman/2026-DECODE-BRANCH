package org.firstinspires.ftc.teamcode.logic;

/**
 * Turns the robot so the shooter points at the goal.
 *
 * <p>Vision first: Limelight tx (degrees, positive = target right of crosshair) is the most direct
 * measurement of aim error and does not depend on odometry. If the tag is not visible it falls back
 * to the odometry heading towards the goal.
 */
public final class AimController {
    public static final class Gains {
        public double kP = 1.8; // turn power per radian of error
        public double kD = 0.08;
        public double maxTurn = 0.6;
        public double minTurn = 0.06;
        public double tolerance = Math.toRadians(2.0);
        /** Extra heading offset (radians) if the shooter is not on the robot's front, e.g. PI. */
        public double shooterHeadingOffset = 0.0;
        /** Camera position relative to the robot centre, inches (forward +, left +). */
        public double cameraForward = 0.0;
        public double cameraLeft = 0.0;
    }

    private final Gains g;
    private final PIDController pid;
    private double lastError;

    public AimController(Gains gains) {
        this.g = gains;
        this.pid = new PIDController(gains.kP, 0, gains.kD);
    }

    /** Heading error (radians, CCW positive) from a Limelight tx reading in degrees. */
    public static double errorFromVision(double txDegrees) {
        return -Math.toRadians(txDegrees);
    }

    /**
     * Heading error (radians, CCW positive) for the SHOOTER (robot centre line) from a Limelight tx when the
     * camera is mounted off-centre.
     *
     * @param txDegrees Limelight tx, + = tag right of the crosshair
     * @param cameraToTagFloorDistance floor distance from the camera to the tag, inches. If it is not a
     *     positive number the uncorrected tx is used.
     */
    public double errorFromVision(double txDegrees, double cameraToTagFloorDistance) {
        double b = errorFromVision(txDegrees);
        if (!MathUtil.isFinite(cameraToTagFloorDistance) || cameraToTagFloorDistance <= 0) return b;
        double fx = g.cameraForward + cameraToTagFloorDistance * Math.cos(b);
        double fy = g.cameraLeft + cameraToTagFloorDistance * Math.sin(b);
        return MathUtil.wrapAngle(Math.atan2(fy, fx) + g.shooterHeadingOffset);
    }

    /** Heading error (radians, CCW positive) from odometry. */
    public double errorFromPose(Pose2d pose, double goalX, double goalY) {
        double want = pose.headingTo(goalX, goalY) + g.shooterHeadingOffset;
        return MathUtil.wrapAngle(want - pose.heading);
    }

    /** @return CCW-positive turn power */
    public double calculate(double headingError, double dt) {
        lastError = headingError;
        if (Math.abs(headingError) <= g.tolerance * 0.5) {
            pid.reset();
            return 0;
        }
        double out = pid.calculate(headingError, dt);
        if (Math.abs(out) < g.minTurn) out = Math.signum(headingError) * g.minTurn;
        return MathUtil.clamp(out, -g.maxTurn, g.maxTurn);
    }

    public boolean onTarget() {
        return Math.abs(lastError) <= g.tolerance;
    }

    public void reset() {
        pid.reset();
        lastError = Double.MAX_VALUE;
    }
}
