package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.Alliance;
import org.firstinspires.ftc.teamcode.logic.ArtifactColor;
import org.firstinspires.ftc.teamcode.logic.FieldGeometry;
import org.firstinspires.ftc.teamcode.logic.MathUtil;
import org.firstinspires.ftc.teamcode.logic.MecanumKinematics;
import org.firstinspires.ftc.teamcode.logic.MecanumOdometry;
import org.firstinspires.ftc.teamcode.logic.Pose2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Physics stand-in for the real robot, used to run the real {@link
 * org.firstinspires.ftc.teamcode.logic.AutoRoutine} end-to-end on a PC.
 *
 * <p>Models: per-wheel first-order motor lag, strafe slip, encoder quantisation, IMU noise and drift,
 * field-wall clamping, a flywheel with first-order spin-up, battery sag and a speed drop on every
 * shot, servo travel time for the sorter and kicker, and a noisy, latency-delayed Limelight tx. It is
 * deliberately pessimistic so passing here leaves margin on the real field.
 */
public final class SimRobot {
    public static final class Params {
        public double maxWheelSpeed = 50; // in/s at full power
        public double wheelTau = 0.12; // s
        public double strafeEfficiency = 0.88; // fraction of commanded strafe actually achieved
        public double trackRadius = 13; // in: wheel speed / angular speed
        public double imuNoiseRad = Math.toRadians(0.15);
        public double imuDriftRadPerSec = Math.toRadians(0.05);
        public double flywheelFreeTps = 2800;
        public double flywheelTau = 0.45;
        public double batteryVolts = 12.6;
        public double batterySagPerAmpProxy = 0.9; // volts lost at full total load
        public double shotSpeedLoss = 0.12; // fraction of flywheel speed lost per launched artifact
        public double sorterServoSpeed = 1.5; // position units / s
        public double kickerServoSpeed = 3.0;
        public double visionNoiseDeg = 0.4;
        public double visionLatency = 0.04;
        public double visionFovDeg = 27; // half-FOV
        public boolean visionAvailable = true;
        public boolean flywheelEncoderWorks = true;
        public boolean drivetrainBlocked = false; // e.g. pinned by another robot
        public long seed = 7;
    }

    /** One artifact leaving the robot. */
    public static final class Shot {
        public final double time;
        public final Pose2d pose;
        public final double flywheelTps;
        public final double targetTps;
        public final ArtifactColor color;
        public final boolean sorterSettled;
        public final boolean legalZone;
        public final double aimErrorDeg;

        Shot(double time, Pose2d pose, double flywheelTps, double targetTps, ArtifactColor color,
                boolean sorterSettled, boolean legalZone, double aimErrorDeg) {
            this.time = time;
            this.pose = pose;
            this.flywheelTps = flywheelTps;
            this.targetTps = targetTps;
            this.color = color;
            this.sorterSettled = sorterSettled;
            this.legalZone = legalZone;
            this.aimErrorDeg = aimErrorDeg;
        }
    }

    public final Params p;
    private final Random rng;
    private final Alliance alliance;

    // truth
    private double x, y, heading;
    private final double[] wheelSpeed = new double[4];
    private final double[] wheelTravel = new double[4];
    private double flywheel;
    private double sorterPos, kickerPos;
    private final ArtifactColor[] slots;
    private double time;
    private double imuBias;
    private double lastTx = Double.NaN;
    private final List<double[]> txHistory = new ArrayList<>();

    // commands
    private double[] wheelCmd = new double[4];
    private double flywheelPower;
    private int sorterSlotCmd;
    private boolean kickerCmd, kickerPrev;
    private double targetTps;

    public final List<Shot> shots = new ArrayList<>();
    public final StringBuilder trace = new StringBuilder("t,x,y,headingDeg,flywheel,target,phase,shots\n");
    public boolean wallContact;

    public SimRobot(Params params, Pose2d start, ArtifactColor[] preload, Alliance alliance) {
        this.p = params;
        this.rng = new Random(params.seed);
        this.alliance = alliance;
        x = start.x;
        y = start.y;
        heading = start.heading;
        slots = preload.clone();
        sorterPos = RobotConfig.SORTER_POSITIONS[RobotConfig.SORTER_START_SLOT];
        kickerPos = RobotConfig.KICKER_REST;
    }

    // ------------------------------------------------------------------ sensors
    public double[] encoderTicks() {
        double[] t = new double[4];
        for (int i = 0; i < 4; i++) t[i] = Math.round(wheelTravel[i] / RobotConfig.DRIVE_INCHES_PER_TICK);
        return t;
    }

    public double imuHeading(double startHeading) {
        // IMU zeroed at the start pose: reports heading relative to it, plus noise and drift.
        return MathUtil.wrapAngle(heading - startHeading + imuBias + rng.nextGaussian() * p.imuNoiseRad);
    }

    public double flywheelEncoder() {
        return p.flywheelEncoderWorks ? flywheel : 0;
    }

    public double batteryVoltage() {
        double load = Math.abs(flywheelPower) * 0.4;
        for (double w : wheelCmd) load += Math.abs(w) * 0.15;
        return p.batteryVolts - p.batterySagPerAmpProxy * MathUtil.clamp(load, 0, 1);
    }

    /** Delayed, noisy Limelight tx to our goal (degrees, + = goal to the right), or null. */
    public Double visionTx() {
        if (!p.visionAvailable) return null;
        double want = Math.atan2(alliance.goalY() - y, alliance.goalX() - x);
        double err = Math.toDegrees(MathUtil.wrapAngle(want - heading));
        txHistory.add(new double[] {time, -err});
        while (txHistory.size() > 1 && txHistory.get(1)[0] <= time - p.visionLatency) txHistory.remove(0);
        double tx = txHistory.get(0)[1];
        if (Math.abs(tx) > p.visionFovDeg) return null;
        lastTx = tx + rng.nextGaussian() * p.visionNoiseDeg;
        return lastTx;
    }

    // ------------------------------------------------------------------ actuators
    public void command(double forward, double left, double turn, double flywheelPower, int sorterSlot,
            boolean kickerFire, double targetTps) {
        MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(forward, left, turn);
        wheelCmd = w.toArray();
        this.flywheelPower = flywheelPower;
        this.sorterSlotCmd = sorterSlot;
        this.kickerCmd = kickerFire;
        this.targetTps = targetTps;
    }

    // ------------------------------------------------------------------ physics
    public void step(double dt, String phase) {
        time += dt;
        imuBias += p.imuDriftRadPerSec * dt;

        for (int i = 0; i < 4; i++) {
            double target = p.drivetrainBlocked ? 0 : wheelCmd[i] * p.maxWheelSpeed * (batteryVoltage() / 12.5);
            wheelSpeed[i] += (target - wheelSpeed[i]) / p.wheelTau * dt;
            wheelTravel[i] += (p.drivetrainBlocked ? wheelCmd[i] * p.maxWheelSpeed * 0.3 : wheelSpeed[i]) * dt; // blocked wheels spin & slip
        }
        double[] robot = MecanumKinematics.wheelDeltasToRobot(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
        double omega = (-wheelSpeed[0] + wheelSpeed[1] - wheelSpeed[2] + wheelSpeed[3]) / 4.0 / p.trackRadius;
        double[] field = MecanumKinematics.robotToField(robot[0], robot[1] * p.strafeEfficiency, heading);
        x += field[0] * dt;
        y += field[1] * dt;
        heading = MathUtil.wrapAngle(heading + omega * dt);

        // Field walls: clamp so the ROTATED footprint stays inside (a turning robot sweeps wider).
        double hl = RobotConfig.ROBOT_LENGTH / 2, hw = RobotConfig.ROBOT_WIDTH / 2;
        double ac = Math.abs(Math.cos(heading)), as = Math.abs(Math.sin(heading));
        double limX = FieldGeometry.HALF_FIELD - (hl * ac + hw * as);
        double limY = FieldGeometry.HALF_FIELD - (hl * as + hw * ac);
        if (Math.abs(x) > limX + 0.25 || Math.abs(y) > limY + 0.25) wallContact = true; // 0.25in = bumper squish
        x = MathUtil.clamp(x, -limX, limX);
        y = MathUtil.clamp(y, -limY, limY);

        double v = batteryVoltage();
        flywheel += (flywheelPower * v / 12.5 * p.flywheelFreeTps - flywheel) / p.flywheelTau * dt;

        double sorterTarget = RobotConfig.SORTER_POSITIONS[sorterSlotCmd];
        sorterPos += MathUtil.clamp(sorterTarget - sorterPos, -p.sorterServoSpeed * dt, p.sorterServoSpeed * dt);
        double kickerTarget = kickerCmd ? RobotConfig.KICKER_FIRE : RobotConfig.KICKER_REST;
        kickerPos += MathUtil.clamp(kickerTarget - kickerPos, -p.kickerServoSpeed * dt, p.kickerServoSpeed * dt);

        // An artifact launches when the kicker reaches the end of its stroke.
        boolean kickerAtFire = Math.abs(kickerPos - RobotConfig.KICKER_FIRE) < 0.02;
        if (kickerAtFire && !kickerPrev) {
            boolean settled = Math.abs(sorterPos - sorterTarget) < 0.02;
            ArtifactColor c = slots[sorterSlotCmd];
            if (c != ArtifactColor.EMPTY) {
                Pose2d pose = getTruePose();
                double want = pose.headingTo(alliance.goalX(), alliance.goalY());
                shots.add(new Shot(time, pose, flywheel, targetTps, c, settled,
                        FieldGeometry.canLaunchFrom(pose, RobotConfig.ROBOT_LENGTH, RobotConfig.ROBOT_WIDTH),
                        Math.toDegrees(MathUtil.wrapAngle(want - heading))));
                slots[sorterSlotCmd] = ArtifactColor.EMPTY;
                flywheel *= (1 - p.shotSpeedLoss);
            }
        }
        kickerPrev = kickerAtFire;

        trace.append(String.format(java.util.Locale.US, "%.3f,%.2f,%.2f,%.1f,%.0f,%.0f,%s,%d%n",
                time, x, y, Math.toDegrees(heading), flywheel, targetTps, phase, shots.size()));
    }

    public Pose2d getTruePose() {
        return new Pose2d(x, y, heading);
    }

    public double getTime() {
        return time;
    }

    /** Odometry exactly as the robot computes it, fed by the simulated sensors. */
    public static MecanumOdometry newOdometry() {
        return new MecanumOdometry(RobotConfig.DRIVE_INCHES_PER_TICK, RobotConfig.LATERAL_MULTIPLIER);
    }
}
