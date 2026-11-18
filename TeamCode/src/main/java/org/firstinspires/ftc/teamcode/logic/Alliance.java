package org.firstinspires.ftc.teamcode.logic;

/** Alliance colour. Red owns the +Y half of the field, Blue the -Y half (FTC SDK DECODE frame). */
public enum Alliance {
    RED(24),
    BLUE(20);

    /** AprilTag ID on the front face of this alliance's GOAL. */
    public final int goalTagId;

    Alliance(int goalTagId) {
        this.goalTagId = goalTagId;
    }

    /** Mirrors a RED-side pose to this alliance (Blue = reflection across the X axis). */
    public Pose2d mirror(Pose2d redPose) {
        if (this == RED) return redPose;
        return new Pose2d(redPose.x, -redPose.y, MathUtil.wrapAngle(-redPose.heading));
    }

    public double goalX() {
        return FieldGeometry.GOAL_X;
    }

    public double goalY() {
        return this == RED ? FieldGeometry.GOAL_Y_RED : -FieldGeometry.GOAL_Y_RED;
    }
}
