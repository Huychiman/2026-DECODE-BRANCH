package org.firstinspires.ftc.teamcode.logic;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ControllersTest {
    @Test
    public void wrapAngle() {
        assertEquals(0, MathUtil.wrapAngle(2 * Math.PI), 1e-12);
        assertEquals(Math.PI, MathUtil.wrapAngle(-Math.PI), 1e-12);
        assertEquals(-Math.PI / 2, MathUtil.wrapAngle(3 * Math.PI / 2), 1e-12);
        assertEquals(0.5, MathUtil.wrapAngle(0.5 + 6 * Math.PI), 1e-9);
    }

    @Test
    public void deadbandRescales() {
        assertEquals(0, MathUtil.deadband(0.04, 0.05), 0);
        assertEquals(1, MathUtil.deadband(1, 0.05), 1e-12);
        assertEquals(-1, MathUtil.deadband(-1, 0.05), 1e-12);
        assertTrue(MathUtil.deadband(0.06, 0.05) > 0);
    }

    @Test
    public void pidProportionalIntegralDerivative() {
        PIDController pid = new PIDController(2, 1, 0.5, 10);
        assertEquals(2 * 1 + 1 * 0.1 + 0, pid.calculate(1, 0.1), 1e-9); // no derivative on first call
        assertEquals(2 * 1 + 1 * 0.2 + 0, pid.calculate(1, 0.1), 1e-9);
        assertEquals(0 + 1 * 0.2 + 0.5 * (-1 / 0.1), pid.calculate(0, 0.1), 1e-9);
        pid.reset();
        assertEquals(2, pid.calculate(1, 0), 1e-9); // dt 0 -> pure P, no NaN
        assertEquals(0, pid.calculate(Double.NaN, 0.1), 0);
    }

    @Test
    public void pidIntegralIsClamped() {
        PIDController pid = new PIDController(0, 1, 0, 0.5);
        double out = 0;
        for (int i = 0; i < 100; i++) out = pid.calculate(1, 0.1);
        assertEquals(0.5, out, 1e-9);
    }

    @Test
    public void driveToPoseConvergesOnIdealRobot() {
        DriveToPose.Gains g = new DriveToPose.Gains();
        DriveToPose d = new DriveToPose(g);
        Pose2d target = Pose2d.degrees(30, -20, 120);
        d.setTarget(target);
        Pose2d p = new Pose2d(0, 0, 0);
        double dt = 0.02, maxSpeed = 50, maxTurn = 4; // in/s, rad/s at full power
        for (int i = 0; i < 500 && !d.atTarget(p); i++) {
            double[] c = d.calculate(p);
            double[] f = MecanumKinematics.robotToField(c[0] * maxSpeed * dt, c[1] * maxSpeed * dt, p.heading);
            p = new Pose2d(p.x + f[0], p.y + f[1], MathUtil.wrapAngle(p.heading + c[2] * maxTurn * dt));
        }
        assertTrue("reached " + p, d.atTarget(p));
    }

    @Test
    public void aimTurnsTowardsTarget() {
        AimController aim = new AimController(new AimController.Gains());
        // Target 10° to the RIGHT -> must turn clockwise (negative).
        assertTrue(aim.calculate(AimController.errorFromVision(10), 0.02) < 0);
        aim.reset();
        assertTrue(aim.calculate(AimController.errorFromVision(-10), 0.02) > 0);
        aim.reset();
        assertEquals(0, aim.calculate(AimController.errorFromVision(0.5), 0.02), 0);
        assertTrue(aim.onTarget());
        // Odometry aim: robot at origin facing +X, goal straight up +Y -> +90° error.
        assertEquals(Math.PI / 2, aim.errorFromPose(new Pose2d(0, 0, 0), 0, 10), 1e-9);
    }

    private static double run(FlywheelController c, double voltage, double seconds, boolean encoderWorks) {
        double speed = 0, dt = 0.02, free = 2800, tau = 0.35;
        for (double t = 0; t < seconds; t += dt) {
            double p = c.update(encoderWorks ? speed : 0, voltage, dt);
            speed += (p * voltage / 12.5 * free - speed) / tau * dt;
        }
        return speed;
    }

    @Test
    public void flywheelReachesTargetAndReportsReady() {
        FlywheelController c = new FlywheelController(new FlywheelController.Gains());
        c.setTarget(2240);
        assertFalse(c.isReady());
        double speed = run(c, 12.5, 3, true);
        assertEquals(2240, speed, 60);
        assertTrue(c.isReady());
    }

    @Test
    public void flywheelHoldsSpeedOnLowBattery() {
        // The old fixed-0.8-power shooter would only reach 2240 * 11.2/12.5 = 2007 on a tired battery.
        FlywheelController c = new FlywheelController(new FlywheelController.Gains());
        c.setTarget(2240);
        assertEquals(2240, run(c, 11.2, 3, true), 60);
        assertTrue(c.isReady());
    }

    @Test
    public void flywheelEncoderFaultFallsBackToTimedOpenLoop() {
        FlywheelController c = new FlywheelController(new FlywheelController.Gains());
        c.setTarget(2240);
        run(c, 12.5, 3, false);
        assertTrue(c.isEncoderFault());
        assertTrue("timer-based ready after fault", c.isReady());
        assertEquals(2240.0 / 2800.0, c.getPower(), 1e-9); // pure feedforward
    }

    @Test
    public void flywheelOffMeansZeroPowerAndNotReady() {
        FlywheelController c = new FlywheelController(new FlywheelController.Gains());
        c.setTarget(0);
        assertEquals(0, c.update(1000, 12.5, 0.02), 0);
        assertFalse(c.isReady());
        c.setTarget(-50);
        assertEquals(0, c.getTarget(), 0);
    }

    @Test
    public void flywheelBadVoltageReadingIsIgnored() {
        FlywheelController c = new FlywheelController(new FlywheelController.Gains());
        c.setTarget(2240);
        double p = c.update(2240, 0.0, 0.02);
        assertEquals(2240.0 / 2800.0, p, 1e-9);
        p = c.update(2240, Double.NaN, 0.02);
        assertEquals(2240.0 / 2800.0, p, 1e-9);
    }
}
