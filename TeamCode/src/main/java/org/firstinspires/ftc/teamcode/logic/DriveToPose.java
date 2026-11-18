package org.firstinspires.ftc.teamcode.logic;

/**
 * Point-to-point holonomic controller: drives straight at a target pose while turning to its heading.
 *
 * <p>Translation uses a P controller on distance (with a minimum power so it does not stall in the
 * last inch) and heading uses a P controller on wrapped heading error. Outputs robot-frame
 * {forward, left, turnCcw} powers ready for {@link MecanumKinematics#robotCentric}.
 */
public final class DriveToPose {
    public static final class Gains {
        public double translationKp = 0.06; // power per inch
        public double headingKp = 1.6; // power per radian
        public double maxPower = 0.7;
        public double maxTurn = 0.6;
        public double minPower = 0.08; // overcome static friction near the target
        public double positionTolerance = 1.5; // inches
        public double headingTolerance = Math.toRadians(3.0);
    }

    private final Gains gains;
    private Pose2d target;

    public DriveToPose(Gains gains) {
        this.gains = gains;
    }

    public void setTarget(Pose2d target) {
        this.target = target;
    }

    public Pose2d getTarget() {
        return target;
    }

    public boolean atTarget(Pose2d pose) {
        if (target == null) return true;
        return pose.distanceTo(target) <= gains.positionTolerance
                && Math.abs(MathUtil.wrapAngle(target.heading - pose.heading)) <= gains.headingTolerance;
    }

    /** @return {forward, left, turnCcw} */
    public double[] calculate(Pose2d pose) {
        if (target == null) return new double[] {0, 0, 0};
        double dx = target.x - pose.x;
        double dy = target.y - pose.y;
        double dist = Math.hypot(dx, dy);

        double speed = 0;
        if (dist > gains.positionTolerance) {
            speed = MathUtil.clamp(dist * gains.translationKp, gains.minPower, gains.maxPower);
        }
        double fx = dist > 1e-9 ? dx / dist * speed : 0;
        double fy = dist > 1e-9 ? dy / dist * speed : 0;
        double[] robot = MecanumKinematics.fieldToRobot(fx, fy, pose.heading);

        double headingError = MathUtil.wrapAngle(target.heading - pose.heading);
        double turn = 0;
        if (Math.abs(headingError) > gains.headingTolerance * 0.5) {
            turn = MathUtil.clamp(headingError * gains.headingKp, -gains.maxTurn, gains.maxTurn);
        }
        return new double[] {robot[0], robot[1], turn};
    }
}
