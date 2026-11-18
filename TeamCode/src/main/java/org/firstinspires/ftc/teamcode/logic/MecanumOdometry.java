package org.firstinspires.ftc.teamcode.logic;

/**
 * Dead-reckoning from the four drive-motor encoders plus the IMU heading.
 *
 * <p>Heading comes from the IMU (far more accurate than wheel-derived heading on mecanum); the wheels
 * only provide translation. Translation is integrated at the midpoint heading of each step. Mecanum
 * wheels slip, so expect a few inches of drift over an autonomous — fine for the short, simple paths
 * the autos drive, and Limelight re-seeds the pose when a goal tag is visible.
 */
public final class MecanumOdometry {
    private final double inchesPerTick;
    private final double lateralMultiplier;

    private double x, y, heading;
    private double[] lastTicks;
    private double headingOffset; // field heading = imuHeading + headingOffset

    /**
     * @param inchesPerTick wheel circumference / encoder ticks per wheel revolution
     * @param lateralMultiplier corrects for mecanum strafe slip (measured strafe distance / reported)
     */
    public MecanumOdometry(double inchesPerTick, double lateralMultiplier) {
        this.inchesPerTick = inchesPerTick;
        this.lateralMultiplier = lateralMultiplier;
    }

    /** Sets the field pose. Call with the current IMU reading so heading stays consistent. */
    public void setPose(Pose2d pose, double imuHeading) {
        x = pose.x;
        y = pose.y;
        heading = MathUtil.wrapAngle(pose.heading);
        headingOffset = pose.heading - imuHeading;
    }

    /** Re-seeds x/y only (e.g. from vision) without touching heading. */
    public void setPosition(double newX, double newY) {
        x = newX;
        y = newY;
    }

    /**
     * @param ticks encoder positions fl, fr, bl, br with "forward = positive" signs
     * @param imuHeading raw IMU yaw in radians (CCW positive)
     */
    public Pose2d update(double[] ticks, double imuHeading) {
        double newHeading = MathUtil.wrapAngle(imuHeading + headingOffset);
        if (lastTicks == null) {
            lastTicks = ticks.clone();
            heading = newHeading;
            return getPose();
        }
        double dfl = (ticks[0] - lastTicks[0]) * inchesPerTick;
        double dfr = (ticks[1] - lastTicks[1]) * inchesPerTick;
        double dbl = (ticks[2] - lastTicks[2]) * inchesPerTick;
        double dbr = (ticks[3] - lastTicks[3]) * inchesPerTick;
        lastTicks = ticks.clone();

        double[] robot = MecanumKinematics.wheelDeltasToRobot(dfl, dfr, dbl, dbr);
        double forward = robot[0];
        double left = robot[1] * lateralMultiplier;

        double midHeading = heading + MathUtil.wrapAngle(newHeading - heading) / 2.0;
        double[] field = MecanumKinematics.robotToField(forward, left, midHeading);
        x += field[0];
        y += field[1];
        heading = newHeading;
        return getPose();
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, heading);
    }
}
