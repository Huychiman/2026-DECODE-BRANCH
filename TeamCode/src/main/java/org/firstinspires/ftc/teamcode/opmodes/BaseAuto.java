package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.Alliance;
import org.firstinspires.ftc.teamcode.logic.AutoPaths;
import org.firstinspires.ftc.teamcode.logic.AutoRoutine;
import org.firstinspires.ftc.teamcode.logic.MathUtil;
import org.firstinspires.ftc.teamcode.logic.Motif;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Autonomous wrapper: wires {@link AutoRoutine} (tested in simulation) to the real hardware.
 *
 * <p>Setup: load sorter slots 0/1/2 as {@link RobotConfig#PRELOAD} says (default GREEN, PURPLE,
 * PURPLE), place the robot on the start pose drawn in the README, INIT, and check the telemetry shows
 * the motif once the obelisk is visible.
 */
public abstract class BaseAuto extends NextFTCOpMode {
    private final Alliance alliance;
    private final AutoPaths.Start start;
    private AutoRoutine routine;
    private long lastNanos;

    protected BaseAuto(Alliance alliance, AutoPaths.Start start) {
        this.alliance = alliance;
        this.start = start;
        addComponents(
                new SubsystemComponent(
                        DriveSubsystem.INSTANCE,
                        IntakeSubsystem.INSTANCE,
                        SorterSubsystem.INSTANCE,
                        KickerSubsystem.INSTANCE,
                        ShooterSubsystem.INSTANCE,
                        VisionSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE);
    }

    @Override
    public void onInit() {
        routine = new AutoRoutine(
                RobotConfig.autoSettings(), alliance, start, RobotConfig.PRELOAD, SorterSubsystem.INSTANCE.getSlot());
        DriveSubsystem.INSTANCE.setPose(routine.getPlan().start);
    }

    @Override
    public void onWaitForStart() {
        Motif m = VisionSubsystem.INSTANCE.getMotif();
        if (m != null) routine.setMotif(m);
        double[] vis = VisionSubsystem.INSTANCE.getFieldPosition();

        telemetry.addData("Auto", "%s from %s", alliance, start);
        telemetry.addData("Start pose", routine.getPlan().start);
        telemetry.addData("Preload slots 0/1/2", "%s / %s / %s",
                RobotConfig.PRELOAD[0], RobotConfig.PRELOAD[1], RobotConfig.PRELOAD[2]);
        telemetry.addData("Limelight", VisionSubsystem.INSTANCE.isConnected() ? "connected" : "NOT FOUND (odometry only)");
        telemetry.addData("Motif", routine.getMotif() == null ? "not seen yet (fires slots in order)" : routine.getMotif());
        if (vis != null) telemetry.addData("Vision position", "(%.1f, %.1f)", vis[0], vis[1]);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Re-assert the start pose in case the robot was nudged during INIT.
        DriveSubsystem.INSTANCE.setPose(routine.getPlan().start);
        lastNanos = System.nanoTime();
    }

    @Override
    public void onUpdate() {
        long now = System.nanoTime();
        double dt = MathUtil.clamp((now - lastNanos) / 1e9, 0, 0.1);
        lastNanos = now;

        DriveSubsystem drive = DriveSubsystem.INSTANCE;
        ShooterSubsystem shooter = ShooterSubsystem.INSTANCE;
        VisionSubsystem vision = VisionSubsystem.INSTANCE;
        vision.setRobotYawDegrees(Math.toDegrees(drive.getPose().heading));

        Motif m = vision.getMotif();
        if (m != null) routine.setMotif(m);
        VisionSubsystem.GoalSighting goal = vision.getGoal(alliance);

        AutoRoutine.Output out = routine.update(dt, drive.getPose(), shooter.isReady(), goal == null ? null : goal.txDegrees);
        drive.drive(out.forward, out.left, out.turn);
        shooter.setTargetVelocity(out.flywheelTps);
        shooter.setHood(out.hood);
        SorterSubsystem.INSTANCE.setSlot(out.sorterSlot);
        KickerSubsystem.INSTANCE.setFiring(out.kickerFire);

        telemetry.addData("Phase", "%s  (%.1fs)", routine.getPhase(), routine.getElapsed());
        telemetry.addData("Pose", drive.getPose());
        telemetry.addData("Shots", "%d fired, %d without ready flywheel", routine.getShotsFired(), routine.getTimedOutShots());
        telemetry.addData("Flywheel", "target %.0f actual %.0f %s", shooter.getTargetVelocity(), shooter.getVelocity(),
                shooter.isEncoderFault() ? "ENCODER FAULT" : "");
        telemetry.addData("Motif", routine.getMotif());
        telemetry.update();
    }

    @Override
    public void onStop() {
        DriveSubsystem.INSTANCE.stop();
        ShooterSubsystem.INSTANCE.stop();
        VisionSubsystem.INSTANCE.stop();
    }
}
