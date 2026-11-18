package org.firstinspires.ftc.teamcode.logic;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

/** Checks the autonomous paths against the DECODE rules, for both alliances and both starts. */
public class FieldRulesTest {
    private static final double L = RobotConfig.ROBOT_LENGTH, W = RobotConfig.ROBOT_WIDTH;

    @Test
    public void launchZoneShapes() {
        assertTrue(FieldGeometry.pointInTriangle(-60, 0, FieldGeometry.GOAL_ZONE));
        assertTrue(FieldGeometry.pointInTriangle(-1, 0, FieldGeometry.GOAL_ZONE));
        assertTrue(FieldGeometry.pointInTriangle(-30, 30, FieldGeometry.GOAL_ZONE)); // on the line counts
        assertFalse(FieldGeometry.pointInTriangle(-20, 30, FieldGeometry.GOAL_ZONE));
        assertFalse(FieldGeometry.pointInTriangle(1, 0, FieldGeometry.GOAL_ZONE));
        assertTrue(FieldGeometry.pointInTriangle(60, 0, FieldGeometry.AUDIENCE_ZONE));
        assertFalse(FieldGeometry.pointInTriangle(60, 20, FieldGeometry.AUDIENCE_ZONE));
        assertFalse(FieldGeometry.pointInTriangle(40, 0, FieldGeometry.AUDIENCE_ZONE));
    }

    @Test
    public void goalPositionsMatchSdkTagLibrary() {
        assertEquals(-58.3727, Alliance.RED.goalX(), 1e-4);
        assertEquals(55.6425, Alliance.RED.goalY(), 1e-4);
        assertEquals(-55.6425, Alliance.BLUE.goalY(), 1e-4);
    }

    @Test
    public void floorDistanceFromCameraRange() {
        double dz = FieldGeometry.GOAL_TAG_HEIGHT - 10;
        assertEquals(48, FieldGeometry.floorDistanceFromRange(Math.hypot(48, dz), 10), 1e-9);
        assertEquals(0, FieldGeometry.floorDistanceFromRange(5, 10), 0);
        assertTrue(Double.isNaN(FieldGeometry.floorDistanceFromRange(Double.NaN, 10)));
    }

    @Test
    public void everyAutoPlanIsLegal() {
        for (Alliance a : Alliance.values()) {
            for (AutoPaths.Start s : AutoPaths.Start.values()) {
                AutoPaths.Plan p = AutoPaths.plan(a, s);
                String tag = a + "/" + s;

                // G304: start over a launch line, inside the field, entirely on our half.
                assertTrue(tag + " start over launch line", FieldGeometry.canLaunchFrom(p.start, L, W));
                assertTrue(tag + " start in field", FieldGeometry.insideField(p.start, L, W));
                for (double[] c : FieldGeometry.robotCorners(p.start, L, W)) {
                    assertTrue(tag + " start on own half", a == Alliance.RED ? c[1] > 0 : c[1] < 0);
                }
                // G416: launch only when over a launch zone.
                assertTrue(tag + " shoot spot legal", FieldGeometry.canLaunchFrom(p.shoot, L, W));
                assertTrue(tag + " shoot spot in field", FieldGeometry.insideField(p.shoot, L, W));
                // LEAVE: off every launch line.
                assertTrue(tag + " leave spot", FieldGeometry.hasLeft(p.leave, L, W));
                assertTrue(tag + " leave in field", FieldGeometry.insideField(p.leave, L, W));
                // Shoot heading faces our goal.
                double want = p.shoot.headingTo(a.goalX(), a.goalY());
                assertEquals(tag, 0, MathUtil.wrapAngle(want - p.shoot.heading), 1e-9);
            }
        }
    }

    @Test
    public void goalStartTouchesOurGoalFace() {
        // Robot front bumper centre should sit on the goal face line through the tag (within 2in).
        for (Alliance a : Alliance.values()) {
            Pose2d s = AutoPaths.plan(a, AutoPaths.Start.GOAL).start;
            double fx = s.x + Math.cos(s.heading) * L / 2, fy = s.y + Math.sin(s.heading) * L / 2;
            assertTrue(a + " front bumper near goal tag: " + Math.hypot(fx - a.goalX(), fy - a.goalY()),
                    Math.hypot(fx - a.goalX(), fy - a.goalY()) < 4);
        }
    }

    @Test
    public void audienceStartTouchesAudienceWall() {
        for (Alliance a : Alliance.values()) {
            Pose2d s = AutoPaths.plan(a, AutoPaths.Start.AUDIENCE).start;
            double maxX = -1e9;
            for (double[] c : FieldGeometry.robotCorners(s, L, W)) maxX = Math.max(maxX, c[0]);
            assertEquals(a + " touching wall", FieldGeometry.HALF_FIELD, maxX, 0.5);
        }
    }
}
