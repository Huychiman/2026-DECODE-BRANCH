package org.firstinspires.ftc.teamcode.logic;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Start / shoot / leave poses for each autonomous. Defined for RED and mirrored for BLUE.
 *
 * <p>Legality (Competition Manual G304, G416, 10.5.3) is checked by the simulation tests:
 * start over a launch line touching our GOAL or the field wall on our half, launch only while
 * over a launch zone, and end AUTO completely outside both zones (LEAVE, 3 pts).
 */
public final class AutoPaths {
    private AutoPaths() {}

    public enum Start {
        /** Front bumper touching the front face of our GOAL, straddling the goal launch line. */
        GOAL,
        /** Back bumper against the audience wall, straddling the small launch zone line. */
        AUDIENCE
    }

    public static final class Plan {
        public final Pose2d start;
        public final Pose2d shoot;
        public final Pose2d leave;

        Plan(Pose2d start, Pose2d shoot, Pose2d leave) {
            this.start = start;
            this.shoot = shoot;
            this.leave = leave;
        }
    }

    public static Plan plan(Alliance alliance, Start start) {
        Pose2d s, shootSpot, leave;
        if (start == Start.GOAL) {
            s = Pose2d.degrees(-52.0, 52.0, 135.0);
            shootSpot = new Pose2d(-30.0, 24.0, 0);
            leave = new Pose2d(-6.0, 40.0, 0);
        } else {
            // Back bumper against the audience wall.
            s = Pose2d.degrees(FieldGeometry.HALF_FIELD - RobotConfig.ROBOT_LENGTH / 2, 12.0, 180.0);
            // 5in off the wall: turning to face the goal swings the corners out to ~11.5in.
            shootSpot = new Pose2d(58.0, 12.0, 0);
            leave = new Pose2d(34.0, 26.0, 0);
        }
        // Face the goal at the shooting spot; keep that heading while leaving (no wasted turning).
        double aim = shootSpot.headingTo(FieldGeometry.GOAL_X, FieldGeometry.GOAL_Y_RED);
        shootSpot = shootSpot.withHeading(aim);
        leave = leave.withHeading(aim);
        return new Plan(alliance.mirror(s), alliance.mirror(shootSpot), alliance.mirror(leave));
    }
}
