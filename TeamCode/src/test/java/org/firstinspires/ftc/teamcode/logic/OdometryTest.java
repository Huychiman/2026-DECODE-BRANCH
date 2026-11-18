package org.firstinspires.ftc.teamcode.logic;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class OdometryTest {
    private static final double IPT = 0.01; // inches per tick

    @Test
    public void firstUpdateOnlyLatchesEncoders() {
        MecanumOdometry o = new MecanumOdometry(IPT, 1.0);
        o.setPose(new Pose2d(5, 6, 0), 0);
        Pose2d p = o.update(new double[] {1000, 2000, 3000, 4000}, 0);
        assertEquals(5, p.x, 1e-9);
        assertEquals(6, p.y, 1e-9);
    }

    @Test
    public void driveForwardAlongHeading() {
        MecanumOdometry o = new MecanumOdometry(IPT, 1.0);
        o.setPose(Pose2d.degrees(0, 0, 90), 0);
        o.update(new double[] {0, 0, 0, 0}, 0);
        Pose2d p = o.update(new double[] {1000, 1000, 1000, 1000}, 0);
        assertEquals(0, p.x, 1e-9);
        assertEquals(10, p.y, 1e-9);
        assertEquals(Math.PI / 2, p.heading, 1e-9);
    }

    @Test
    public void strafeAppliesLateralMultiplier() {
        MecanumOdometry o = new MecanumOdometry(IPT, 1.2);
        o.setPose(new Pose2d(0, 0, 0), 0);
        o.update(new double[] {0, 0, 0, 0}, 0);
        Pose2d p = o.update(new double[] {-1000, 1000, 1000, -1000}, 0); // strafe left
        assertEquals(0, p.x, 1e-9);
        assertEquals(12, p.y, 1e-9);
    }

    @Test
    public void headingComesFromImuWithOffset() {
        MecanumOdometry o = new MecanumOdometry(IPT, 1.0);
        // IMU currently reads 0.3 rad but the robot faces 180° on the field.
        o.setPose(Pose2d.degrees(0, 0, 180), 0.3);
        Pose2d p = o.update(new double[] {0, 0, 0, 0}, 0.3);
        assertEquals(Math.PI, Math.abs(p.heading), 1e-9);
        p = o.update(new double[] {0, 0, 0, 0}, 0.3 + Math.PI / 2);
        assertEquals(-Math.PI / 2, p.heading, 1e-9);
    }

    @Test
    public void arcIsIntegratedAtMidpointHeading() {
        // Drive a quarter circle of radius 20 in 200 steps; midpoint integration should land close.
        MecanumOdometry o = new MecanumOdometry(IPT, 1.0);
        o.setPose(new Pose2d(0, 0, 0), 0);
        double[] ticks = {0, 0, 0, 0};
        o.update(ticks, 0);
        int n = 200;
        double arc = Math.PI / 2 * 20;
        for (int i = 1; i <= n; i++) {
            for (int k = 0; k < 4; k++) ticks[k] += arc / n / IPT;
            o.update(ticks, Math.PI / 2 * i / n);
        }
        Pose2d p = o.getPose();
        assertEquals(20, p.x, 0.01);
        assertEquals(20, p.y, 0.01);
    }
}
