package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.Alliance;
import org.firstinspires.ftc.teamcode.logic.ArtifactColor;
import org.firstinspires.ftc.teamcode.logic.AutoPaths;
import org.firstinspires.ftc.teamcode.logic.AutoRoutine;
import org.firstinspires.ftc.teamcode.logic.FieldGeometry;
import org.firstinspires.ftc.teamcode.logic.FlywheelController;
import org.firstinspires.ftc.teamcode.logic.MecanumOdometry;
import org.firstinspires.ftc.teamcode.logic.Motif;
import org.firstinspires.ftc.teamcode.logic.Pose2d;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

/**
 * End-to-end autonomous simulation: the production {@link AutoRoutine}, {@link MecanumOdometry} and
 * {@link FlywheelController} run against {@link SimRobot} physics for a full 30s AUTO, with loop-time
 * jitter, in every alliance x start x motif combination plus fault scenarios. Traces are written to
 * TeamCode/build/sim/ for plotting.
 */
public class AutoSimulationTest {
    private static final double AUTO_SECONDS = 30.0;

    static final class Result {
        SimRobot sim;
        AutoRoutine routine;
        double finishedAt = Double.NaN;
    }

    static Result run(Alliance alliance, AutoPaths.Start start, Motif motif, SimRobot.Params params, String traceName) {
        return run(RobotConfig.autoSettings(), alliance, start, motif, params, traceName);
    }

    static Result run(AutoRoutine.Settings settings, Alliance alliance, AutoPaths.Start start, Motif motif,
            SimRobot.Params params, String traceName) {
        AutoRoutine routine = new AutoRoutine(settings, alliance, start, RobotConfig.PRELOAD,
                RobotConfig.SORTER_START_SLOT);
        Pose2d startPose = routine.getPlan().start;
        SimRobot sim = new SimRobot(params, startPose, RobotConfig.PRELOAD, alliance);
        MecanumOdometry odo = SimRobot.newOdometry();
        odo.setPose(startPose, sim.imuHeading(startPose.heading));
        FlywheelController fly = new FlywheelController(RobotConfig.flywheelGains());
        if (motif != null) routine.setMotif(motif);

        Random jitter = new Random(params.seed + 1);
        Result r = new Result();
        r.sim = sim;
        r.routine = routine;
        double t = 0;
        while (t < AUTO_SECONDS) {
            double dt = 0.012 + jitter.nextDouble() * 0.016; // 12-28ms loops like a real hub
            Pose2d pose = odo.update(sim.encoderTicks(), sim.imuHeading(startPose.heading));
            AutoRoutine.Output out = routine.update(dt, pose, fly.isReady(), sim.visionTx());
            fly.setTarget(out.flywheelTps);
            double power = fly.update(sim.flywheelEncoder(), sim.batteryVoltage(), dt);
            sim.command(out.forward, out.left, out.turn, power, out.sorterSlot, out.kickerFire, out.flywheelTps);
            sim.step(dt, routine.getPhase().name());
            t += dt;
            if (routine.getPhase() == AutoRoutine.Phase.DONE && Double.isNaN(r.finishedAt)) r.finishedAt = t;
        }
        if (traceName != null) writeTrace(traceName, sim.trace.toString());
        return r;
    }

    private static void writeTrace(String name, String csv) {
        File dir = new File("build/sim");
        if (!dir.exists() && !dir.mkdirs()) return;
        try (FileWriter w = new FileWriter(new File(dir, name + ".csv"))) {
            w.write(csv);
        } catch (IOException ignored) {
            // Traces are a convenience; never fail a test because of them.
        }
    }

    private static void assertScoringAuto(String tag, Result r, Motif motif) {
        SimRobot sim = r.sim;
        assertEquals(tag + " shots", 3, sim.shots.size());
        for (int i = 0; i < sim.shots.size(); i++) {
            SimRobot.Shot s = sim.shots.get(i);
            String st = tag + " shot " + i;
            assertTrue(st + " launched from a launch zone (G416) at " + s.pose, s.legalZone);
            assertTrue(st + " sorter settled before kick", s.sorterSettled);
            assertTrue(st + " flywheel at speed: " + s.flywheelTps + " vs " + s.targetTps,
                    Math.abs(s.flywheelTps - s.targetTps) <= 120);
            assertTrue(st + " aimed at goal, error " + s.aimErrorDeg + "°", Math.abs(s.aimErrorDeg) <= 4.0);
            if (motif != null) assertEquals(st + " matches motif " + motif, motif.colorAt(i), s.color);
        }
        Pose2d end = sim.getTruePose();
        assertTrue(tag + " LEAVE: robot off every launch line at 30s, true pose " + end,
                FieldGeometry.hasLeft(end, RobotConfig.ROBOT_LENGTH, RobotConfig.ROBOT_WIDTH));
        assertTrue(tag + " routine finished before 30s", r.finishedAt < AUTO_SECONDS);
        assertTrue(tag + " never hit a field wall", !sim.wallContact);
    }

    @Test
    public void everyAllianceStartAndMotifScoresAndLeaves() {
        for (Alliance a : Alliance.values()) {
            for (AutoPaths.Start s : AutoPaths.Start.values()) {
                for (Motif m : Motif.values()) {
                    String tag = a + "/" + s + "/" + m;
                    Result r = run(a, s, m, new SimRobot.Params(), (a + "_" + s + "_" + m).toLowerCase());
                    assertScoringAuto(tag, r, m);
                }
            }
        }
    }

    private static double meanAbsAimError(Result r) {
        double sum = 0;
        for (SimRobot.Shot s : r.sim.shots) sum += Math.abs(s.aimErrorDeg);
        return sum / r.sim.shots.size();
    }

    @Test
    public void offCentreLimelightIsCorrectedForParallax() {
        // CAD: the Limelight is 3.7in right of the shooter. Aiming the CAMERA at the tag would put every
        // goal-start shot ~5 degrees off; the parallax correction must bring the SHOOTER onto the goal.
        for (Alliance a : Alliance.values()) {
            Result fixed = run(a, AutoPaths.Start.GOAL, Motif.GPP, new SimRobot.Params(), null);
            assertEquals(3, fixed.sim.shots.size());
            assertTrue(a + " corrected aim error " + meanAbsAimError(fixed), meanAbsAimError(fixed) < 2.0);

            AutoRoutine.Settings naive = RobotConfig.autoSettings();
            naive.aim.cameraForward = 0;
            naive.aim.cameraLeft = 0;
            Result uncorrected = run(naive, a, AutoPaths.Start.GOAL, Motif.GPP, new SimRobot.Params(), null);
            assertTrue(a + " uncorrected aim error should be large, was " + meanAbsAimError(uncorrected),
                    meanAbsAimError(uncorrected) > 3.0);
        }
    }

    @Test
    public void unknownMotifStillScoresAllThree() {
        Result r = run(Alliance.RED, AutoPaths.Start.GOAL, null, new SimRobot.Params(), null);
        assertScoringAuto("no motif", r, null);
    }

    @Test
    public void worksWithoutLimelight() {
        SimRobot.Params p = new SimRobot.Params();
        p.visionAvailable = false;
        for (AutoPaths.Start s : AutoPaths.Start.values()) {
            assertScoringAuto("no vision " + s, run(Alliance.BLUE, s, Motif.PGP, p, null), Motif.PGP);
        }
    }

    @Test
    public void flywheelEncoderUnpluggedStillShootsAndLeaves() {
        SimRobot.Params p = new SimRobot.Params();
        p.flywheelEncoderWorks = false;
        Result r = run(Alliance.RED, AutoPaths.Start.GOAL, Motif.GPP, p, "fault_flywheel_encoder");
        assertEquals(3, r.sim.shots.size());
        for (SimRobot.Shot s : r.sim.shots) assertTrue("legal", s.legalZone);
        assertTrue(FieldGeometry.hasLeft(r.sim.getTruePose(), RobotConfig.ROBOT_LENGTH, RobotConfig.ROBOT_WIDTH));
    }

    @Test
    public void heavySlowFlywheelWaitsForSpeedBeforeEachShot() {
        // Slow spin-up and a big speed drop per shot: only the "ready" gate keeps shots consistent.
        SimRobot.Params p = new SimRobot.Params();
        p.flywheelTau = 1.1;
        p.shotSpeedLoss = 0.25;
        for (AutoPaths.Start s : AutoPaths.Start.values()) {
            Result r = run(Alliance.RED, s, Motif.PGP, p, null);
            assertScoringAuto("heavy flywheel " + s, r, Motif.PGP);
            assertEquals("no shot needed the timeout", 0, r.routine.getTimedOutShots());
        }
    }

    @Test
    public void lowBatteryAndBadStrafeSlipStillScore() {
        SimRobot.Params p = new SimRobot.Params();
        p.batteryVolts = 11.6;
        p.strafeEfficiency = 0.75;
        p.maxWheelSpeed = 40;
        for (Alliance a : Alliance.values()) {
            for (AutoPaths.Start s : AutoPaths.Start.values()) {
                assertScoringAuto("stressed " + a + "/" + s, run(a, s, Motif.PPG, p, null), Motif.PPG);
            }
        }
    }

    @Test
    public void manyRandomSeedsAllPass() {
        for (long seed = 100; seed < 140; seed++) {
            SimRobot.Params p = new SimRobot.Params();
            p.seed = seed;
            Alliance a = seed % 2 == 0 ? Alliance.RED : Alliance.BLUE;
            AutoPaths.Start s = seed % 4 < 2 ? AutoPaths.Start.GOAL : AutoPaths.Start.AUDIENCE;
            Motif m = Motif.values()[(int) (seed % 3)];
            assertScoringAuto("seed " + seed, run(a, s, m, p, null), m);
        }
    }

    @Test
    public void pinnedRobotGivesUpShootingAtDeadlineAndStops() {
        SimRobot.Params p = new SimRobot.Params();
        p.drivetrainBlocked = true;
        Result r = run(Alliance.RED, AutoPaths.Start.GOAL, Motif.GPP, p, null);
        // It cannot move, but it must not hang: every phase must be bounded by timeouts.
        assertEquals(AutoRoutine.Phase.DONE, r.routine.getPhase());
        for (SimRobot.Shot s : r.sim.shots) assertTrue("never shoots illegally", s.legalZone);
    }
}
