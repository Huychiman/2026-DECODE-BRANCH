package org.firstinspires.ftc.teamcode.logic;

import static org.firstinspires.ftc.teamcode.logic.ArtifactColor.EMPTY;
import static org.firstinspires.ftc.teamcode.logic.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.logic.ArtifactColor.PURPLE;
import static org.firstinspires.ftc.teamcode.logic.ArtifactColor.UNKNOWN;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class ScoringLogicTest {
    @Test
    public void motifTagsMatchSdkLibrary() {
        assertEquals(Motif.GPP, Motif.fromTagId(21));
        assertEquals(Motif.PGP, Motif.fromTagId(22));
        assertEquals(Motif.PPG, Motif.fromTagId(23));
        assertNull(Motif.fromTagId(20));
        assertNull(Motif.fromTagId(24));
        assertEquals(GREEN, Motif.PGP.colorAt(1));
        assertEquals(GREEN, Motif.PGP.colorAt(4)); // repeats along the 9-index ramp
        assertEquals(GREEN, Motif.PPG.colorAt(8));
    }

    @Test
    public void allianceGoalTagsAndMirror() {
        assertEquals(24, Alliance.RED.goalTagId);
        assertEquals(20, Alliance.BLUE.goalTagId);
        Pose2d b = Alliance.BLUE.mirror(Pose2d.degrees(10, 20, 30));
        assertEquals(10, b.x, 0);
        assertEquals(-20, b.y, 0);
        assertEquals(Math.toRadians(-30), b.heading, 1e-12);
        assertTrue(Alliance.RED.goalY() > 0 && Alliance.BLUE.goalY() < 0);
    }

    @Test
    public void shotTableInterpolatesAndClamps() {
        ShotTable t = new ShotTable().add(60, 2000, 0.6).add(30, 1500, 0.3).add(90, 2600, 0.8);
        assertEquals(1500, t.lookup(10).flywheelTps, 0);
        assertEquals(2600, t.lookup(200).flywheelTps, 0);
        assertEquals(1750, t.lookup(45).flywheelTps, 1e-9);
        assertEquals(0.45, t.lookup(45).hood, 1e-9);
        assertEquals(2300, t.lookup(75).flywheelTps, 1e-9);
        assertEquals(1500, t.lookup(Double.NaN).flywheelTps, 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void shotTableRejectsDuplicates() {
        new ShotTable().add(30, 1, 0.5).add(30, 2, 0.5);
    }

    @Test(expected = IllegalStateException.class)
    public void emptyShotTableFailsLoudly() {
        new ShotTable().lookup(10);
    }

    @Test
    public void configuredShotTableIsSane() {
        ShotTable t = RobotConfig.shotTable();
        assertTrue(t.size() >= 2);
        for (ShotTable.Shot s : t.entries()) {
            assertTrue(s.flywheelTps > 0 && s.flywheelTps <= 2800);
            assertTrue(s.hood >= RobotConfig.HOOD_MIN && s.hood <= RobotConfig.HOOD_MAX);
        }
    }

    @Test
    public void firePlannerMatchesEveryMotifWithStandardPreload() {
        ArtifactColor[] preload = {GREEN, PURPLE, PURPLE};
        for (Motif m : Motif.values()) {
            List<Integer> order = FirePlanner.plan(preload, m, 0, 0);
            assertEquals(3, order.size());
            for (int i = 0; i < 3; i++) assertEquals(m + " shot " + i, m.colorAt(i), preload[order.get(i)]);
        }
    }

    @Test
    public void firePlannerContinuesMotifFromRampCount() {
        // Two already on the ramp -> next needed index 2.
        List<Integer> order = FirePlanner.plan(new ArtifactColor[] {PURPLE, GREEN, PURPLE}, Motif.PPG, 2, 0);
        assertEquals(Integer.valueOf(1), order.get(0)); // GREEN first (index 2 of PPG)
    }

    @Test
    public void firePlannerWithoutMotifMinimisesTravelAndSkipsEmpty() {
        assertEquals(Arrays.asList(1, 0, 2), FirePlanner.plan(new ArtifactColor[] {UNKNOWN, UNKNOWN, UNKNOWN}, null, 0, 1));
        assertEquals(Arrays.asList(2, 1, 0), FirePlanner.plan(new ArtifactColor[] {UNKNOWN, UNKNOWN, UNKNOWN}, null, 0, 2));
        assertEquals(Arrays.asList(0, 2), FirePlanner.plan(new ArtifactColor[] {PURPLE, EMPTY, GREEN}, null, 0, 0));
        assertTrue(FirePlanner.plan(new ArtifactColor[] {EMPTY, EMPTY, EMPTY}, Motif.GPP, 0, 0).isEmpty());
    }

    @Test
    public void sequencerFiresEachSlotOnceWithServoTimes() {
        ShotSequencer.Timing timing = new ShotSequencer.Timing();
        ShotSequencer s = new ShotSequencer(timing, 0);
        s.start(Arrays.asList(0, 1, 2));
        int kicks = 0;
        boolean wasFiring = false;
        double t = 0;
        while (!s.isDone() && t < 10) {
            s.update(0.01, true);
            t += 0.01;
            if (s.isKickerFiring() && !wasFiring) kicks++;
            wasFiring = s.isKickerFiring();
        }
        assertTrue(s.isDone());
        assertEquals(3, kicks);
        assertEquals(3, s.getShotsFired());
        assertEquals(0, s.getTimedOutShots());
        assertEquals(2, s.getSorterSlot());
        double minimum = 3 * (timing.feedSeconds + timing.retractSeconds) + 2 * timing.sorterSettleSeconds;
        assertTrue("took " + t, t >= minimum - 0.05 && t <= minimum + 0.2);
    }

    @Test
    public void sequencerWaitsForFlywheelThenTimesOut() {
        ShotSequencer.Timing timing = new ShotSequencer.Timing();
        ShotSequencer s = new ShotSequencer(timing, 0);
        s.start(Arrays.asList(0));
        for (int i = 0; i < 150; i++) s.update(0.01, false); // 1.5s, timeout is 2.0s
        assertFalse(s.isKickerFiring());
        for (int i = 0; i < 60; i++) s.update(0.01, false);
        assertTrue(s.isKickerFiring() || s.getShotsFired() == 1);
        for (int i = 0; i < 100; i++) s.update(0.01, false);
        assertTrue(s.isDone());
        assertEquals(1, s.getTimedOutShots());
    }

    @Test
    public void sequencerTwoSlotMoveWaitsLonger() {
        ShotSequencer.Timing timing = new ShotSequencer.Timing();
        ShotSequencer s = new ShotSequencer(timing, 0);
        s.start(Arrays.asList(2));
        double t = 0;
        while (!s.isKickerFiring()) {
            s.update(0.01, true);
            t += 0.01;
        }
        assertEquals(2 * timing.sorterSettleSeconds, t, 0.03);
    }

    @Test
    public void sequencerCancelStopsKickingAndFreesSorter() {
        ShotSequencer s = new ShotSequencer(new ShotSequencer.Timing(), 0);
        s.start(Arrays.asList(0, 1));
        for (int i = 0; i < 5; i++) s.update(0.01, true);
        assertTrue(s.isKickerFiring());
        s.cancel();
        assertFalse(s.isKickerFiring());
        assertFalse(s.isBusy());
        s.setSorterSlot(2);
        assertEquals(2, s.getSorterSlot());
    }
}
