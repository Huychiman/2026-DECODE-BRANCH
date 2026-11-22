package org.firstinspires.ftc.teamcode.logic;

import java.util.List;

/**
 * Hardware-free autonomous: drive to the shooting spot (flywheel pre-spinning), aim, fire the three
 * preloads in MOTIF order, then LEAVE. The OpMode feeds sensor readings in and copies the outputs to
 * the hardware; the simulation tests drive exactly the same class.
 *
 * <p>Time guard: whatever happens (blocked path, flywheel never ready, vision junk) the robot
 * abandons shooting at {@link Settings#leaveDeadlineSeconds} so it still scores LEAVE.
 */
public final class AutoRoutine {
    public static final class Settings {
        public DriveToPose.Gains drive = new DriveToPose.Gains();
        public AimController.Gains aim = new AimController.Gains();
        public ShotSequencer.Timing shotTiming = new ShotSequencer.Timing();
        public ShotTable shotTable;
        public double driveTimeoutSeconds = 4.0;
        public double aimTimeoutSeconds = 1.5;
        public double aimSettleSeconds = 0.15;
        public double leaveDeadlineSeconds = 24.0;
    }

    public enum Phase {
        DRIVE_TO_SHOOT,
        AIM,
        SHOOT,
        LEAVE,
        DONE
    }

    /** Commands for the hardware, refreshed by every {@link #update}. */
    public static final class Output {
        public double forward, left, turn;
        public double flywheelTps;
        public double hood;
        public boolean kickerFire;
        public int sorterSlot;
    }

    private final Settings s;
    private final Alliance alliance;
    private final AutoPaths.Plan plan;
    private final DriveToPose driver;
    private final AimController aim;
    private final ShotSequencer sequencer;
    private final ArtifactColor[] preload;
    private final Output out = new Output();
    private final ShotTable.Shot plannedShot;

    private Motif motif;
    private Phase phase = Phase.DRIVE_TO_SHOOT;
    private double elapsed;
    private double phaseTime;
    private double aimedFor;
    private List<Integer> fireOrder;

    public AutoRoutine(
            Settings settings, Alliance alliance, AutoPaths.Start start, ArtifactColor[] preload, int initialSlot) {
        this.s = settings;
        this.alliance = alliance;
        this.plan = AutoPaths.plan(alliance, start);
        this.driver = new DriveToPose(settings.drive);
        this.aim = new AimController(settings.aim);
        this.sequencer = new ShotSequencer(settings.shotTiming, initialSlot);
        this.preload = preload.clone();
        this.plannedShot = settings.shotTable.lookup(FieldGeometry.distanceToGoal(plan.shoot, alliance));
        driver.setTarget(plan.shoot);
        out.sorterSlot = initialSlot;
    }

    public AutoPaths.Plan getPlan() {
        return plan;
    }

    /** Motif seen on the obelisk. Can be set any time before the volley starts. */
    public void setMotif(Motif m) {
        if (fireOrder == null) motif = m;
    }

    public Motif getMotif() {
        return motif;
    }

    /**
     * @param dt seconds since last update
     * @param pose current odometry pose
     * @param flywheelReady flywheel at target speed
     * @param visionTxDeg Limelight tx to OUR goal tag, or null if not visible
     */
    public Output update(double dt, Pose2d pose, boolean flywheelReady, Double visionTxDeg) {
        elapsed += dt;
        phaseTime += dt;
        out.forward = out.left = out.turn = 0;

        boolean shootingPhase = phase == Phase.DRIVE_TO_SHOOT || phase == Phase.AIM || phase == Phase.SHOOT;
        if (shootingPhase && elapsed >= s.leaveDeadlineSeconds) {
            sequencer.cancel();
            enter(Phase.LEAVE);
        }

        // Distance-based shot: start spinning immediately so the wheel is ready on arrival.
        double liveDistance = FieldGeometry.distanceToGoal(pose, alliance);
        ShotTable.Shot shot = phase == Phase.DRIVE_TO_SHOOT ? plannedShot : s.shotTable.lookup(liveDistance);
        boolean spin = phase != Phase.LEAVE && phase != Phase.DONE;
        out.flywheelTps = spin ? shot.flywheelTps : 0;
        out.hood = shot.hood;

        switch (phase) {
            case DRIVE_TO_SHOOT: {
                double[] cmd = driver.calculate(pose);
                set(cmd);
                if (driver.atTarget(pose) || phaseTime >= s.driveTimeoutSeconds) {
                    aim.reset();
                    enter(Phase.AIM);
                }
                break;
            }
            case AIM: {
                double err = aimError(pose, visionTxDeg);
                out.turn = aim.calculate(err, dt);
                aimedFor = aim.onTarget() ? aimedFor + dt : 0;
                if (aimedFor >= s.aimSettleSeconds || phaseTime >= s.aimTimeoutSeconds) {
                    fireOrder = FirePlanner.plan(preload, motif, 0, sequencer.getSorterSlot());
                    sequencer.start(fireOrder);
                    enter(Phase.SHOOT);
                }
                break;
            }
            case SHOOT: {
                // Keep holding aim while firing (recoil and kicker nudges the robot).
                out.turn = aim.calculate(aimError(pose, visionTxDeg), dt);
                sequencer.update(dt, flywheelReady);
                if (sequencer.isDone()) {
                    driver.setTarget(plan.leave);
                    enter(Phase.LEAVE);
                }
                break;
            }
            case LEAVE: {
                if (driver.getTarget() != plan.leave) driver.setTarget(plan.leave);
                set(driver.calculate(pose));
                if (driver.atTarget(pose) || phaseTime >= s.driveTimeoutSeconds + 2.0) enter(Phase.DONE);
                break;
            }
            case DONE:
            default:
                break;
        }
        out.kickerFire = sequencer.isKickerFiring();
        out.sorterSlot = sequencer.getSorterSlot();
        return out;
    }

    private double aimError(Pose2d pose, Double tx) {
        double odoErr = aim.errorFromPose(pose, alliance.goalX(), alliance.goalY());
        if (tx != null && MathUtil.isFinite(tx)) {
            // Off-centre camera: correct tx for parallax using the odometry distance to the goal.
            double camDist = FieldGeometry.distanceToGoal(pose, alliance) - s.aim.cameraForward;
            double visErr = aim.errorFromVision(tx, camDist);
            // Reject a reading wildly different from odometry (wrong tag, reflection).
            if (Math.abs(MathUtil.wrapAngle(visErr - odoErr)) < Math.toRadians(25)) return visErr;
        }
        return odoErr;
    }

    private void set(double[] cmd) {
        out.forward = cmd[0];
        out.left = cmd[1];
        out.turn = cmd[2];
    }

    private void enter(Phase p) {
        phase = p;
        phaseTime = 0;
        aimedFor = 0;
    }

    public Phase getPhase() {
        return phase;
    }

    public int getShotsFired() {
        return sequencer.getShotsFired();
    }

    public int getTimedOutShots() {
        return sequencer.getTimedOutShots();
    }

    public List<Integer> getFireOrder() {
        return fireOrder;
    }

    public double getElapsed() {
        return elapsed;
    }
}
