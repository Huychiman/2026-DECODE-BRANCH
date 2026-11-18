package org.firstinspires.ftc.teamcode.logic;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

import java.util.Random;

public class MecanumTest {
    private static final double E = 1e-9;

    @Test
    public void forwardDrivesAllWheelsForward() {
        MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(1, 0, 0);
        assertArrayEquals(new double[] {1, 1, 1, 1}, w.toArray(), E);
    }

    @Test
    public void strafeLeftUsesXPattern() {
        MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(0, 1, 0);
        assertArrayEquals(new double[] {-1, 1, 1, -1}, w.toArray(), E);
    }

    @Test
    public void counterClockwiseTurnSpinsRightSideForward() {
        MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(0, 0, 1);
        assertArrayEquals(new double[] {-1, 1, -1, 1}, w.toArray(), E);
    }

    @Test
    public void outputIsNormalisedAndKeepsRatios() {
        MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(1, 1, 1);
        assertTrue(w.maxAbs() <= 1 + E);
        assertEquals(1.0, w.maxAbs(), E);
    }

    @Test
    public void wheelKinematicsInvertsTheMix() {
        Random r = new Random(1);
        for (int i = 0; i < 500; i++) {
            double f = r.nextDouble() - 0.5, l = r.nextDouble() - 0.5;
            MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(f, l, 0);
            double[] back = MecanumKinematics.wheelDeltasToRobot(w.fl, w.fr, w.bl, w.br);
            assertEquals(f, back[0], 1e-9);
            assertEquals(l, back[1], 1e-9);
        }
    }

    @Test
    public void fieldRobotRotationRoundTrips() {
        Random r = new Random(2);
        for (int i = 0; i < 500; i++) {
            double x = r.nextGaussian(), y = r.nextGaussian(), h = (r.nextDouble() - 0.5) * 10;
            double[] rob = MecanumKinematics.fieldToRobot(x, y, h);
            double[] fld = MecanumKinematics.robotToField(rob[0], rob[1], h);
            assertEquals(x, fld[0], 1e-9);
            assertEquals(y, fld[1], 1e-9);
        }
        // Robot facing +Y (90°): field +Y is robot forward, field +X is robot right (= -left).
        double[] rob = MecanumKinematics.fieldToRobot(0, 1, Math.PI / 2);
        assertEquals(1, rob[0], E);
        assertEquals(0, rob[1], E);
        rob = MecanumKinematics.fieldToRobot(1, 0, Math.PI / 2);
        assertEquals(0, rob[0], E);
        assertEquals(-1, rob[1], E);
    }

    /**
     * The critical safety check of the rewrite: for every stick input, the new TeleOp must put the
     * SAME physical power on every wheel as the competition TeleOp did.
     *
     * <p>Competition: NextFTC MecanumDriverControlled (decompiled from dev.nextftc:hardware:1.1.0)
     * fed with (drive = lsy*0.9, strafe = -lsx*0.9, turn = -rsx*0.9) and the LEFT motors reversed.
     */
    @Test
    public void newDriveMatchesCompetitionDriveForEveryStickInput() {
        Random r = new Random(42);
        for (int i = 0; i < 20000; i++) {
            double lsx = r.nextDouble() * 2 - 1, lsy = r.nextDouble() * 2 - 1, rsx = r.nextDouble() * 2 - 1;
            if (i < 27) { // also hit every corner/axis exactly
                lsx = (i % 3) - 1;
                lsy = ((i / 3) % 3) - 1;
                rsx = ((i / 9) % 3) - 1;
            }
            double[] oldPhysical = competitionPhysicalPowers(lsx, lsy, rsx);

            // New TeleOp mapping (BaseTeleOp without deadband).
            double s = RobotConfig.DRIVER_MAX_POWER;
            double forward = -lsy * s;
            double left = -lsx * s * RobotConfig.STRAFE_MULTIPLIER;
            double turn = -rsx * s;
            MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(forward, left, turn);
            double ld = RobotConfig.LEFT_REVERSED ? -1 : 1;
            double rd = RobotConfig.RIGHT_REVERSED ? -1 : 1;
            double[] newPhysical = {w.fl * ld, w.fr * rd, w.bl * ld, w.br * rd};

            assertArrayEquals("sticks " + lsx + "," + lsy + "," + rsx, oldPhysical, newPhysical, 1e-12);
        }
    }

    private static double[] competitionPhysicalPowers(double lsx, double lsy, double rsx) {
        double drive = lsy * 0.9, strafe = -lsx * 0.9, turn = -rsx * 0.9;
        double x = strafe * 1.1, y = drive, h = turn;
        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(h), 1.0);
        double fl = (y + x + h) / d, fr = (y - x - h) / d, bl = (y - x + h) / d, br = (y + x - h) / d;
        // Competition directions: leftFront/leftBack reversed, right side forward.
        return new double[] {-fl, fr, -bl, br};
    }
}
