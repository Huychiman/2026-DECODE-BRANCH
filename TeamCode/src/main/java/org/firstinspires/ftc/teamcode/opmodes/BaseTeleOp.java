package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.AimController;
import org.firstinspires.ftc.teamcode.logic.Alliance;
import org.firstinspires.ftc.teamcode.logic.ArtifactColor;
import org.firstinspires.ftc.teamcode.logic.FieldGeometry;
import org.firstinspires.ftc.teamcode.logic.FirePlanner;
import org.firstinspires.ftc.teamcode.logic.MathUtil;
import org.firstinspires.ftc.teamcode.logic.MecanumKinematics;
import org.firstinspires.ftc.teamcode.logic.Motif;
import org.firstinspires.ftc.teamcode.logic.ShotSequencer;
import org.firstinspires.ftc.teamcode.logic.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Driver-controlled period. Controls (gamepad1; gamepad2 can run every mechanism button too):
 *
 * <pre>
 *  Left stick ........ drive / strafe          Right stick X .. turn
 *  Left trigger ...... slow mode               Right trigger .. auto-aim at goal (Limelight)
 *  Triangle / Y ...... intake on/off           Cross / A (hold) eject
 *  Square / X ........ sorter previous slot    Circle / B ..... sorter next slot
 *  Right bumper (hold) flywheel on             Left bumper (hold) kicker fire (manual shot)
 *  D-pad up / down ... hood trim               D-pad left ..... fire all loaded slots (auto volley)
 *  D-pad right ....... cancel volley           Options ........ reset field-centric heading
 *  Share ............. toggle field-centric (starts robot-centric, like the competition code)
 * </pre>
 *
 * Every pre-existing button does what it did at the first competition.
 */
public abstract class BaseTeleOp extends NextFTCOpMode {
    private final Alliance alliance; // null = aim at whichever goal is visible

    private final ShotTable shotTable = RobotConfig.shotTable();
    private final AimController aim = new AimController(RobotConfig.aimGains());
    private ShotSequencer sequencer;

    private boolean flywheelHeld;
    private boolean kickerHeld;
    private boolean fieldCentric;
    private double driverHeading;
    private double hoodTrim;
    private double lastGoalDistance = Double.NaN;
    private double sinceGoalSeen = 99;
    private Motif motif;
    private long lastNanos;
    private double loopMs;

    protected BaseTeleOp(Alliance alliance) {
        this.alliance = alliance;
        addComponents(
                new SubsystemComponent(
                        DriveSubsystem.INSTANCE,
                        IntakeSubsystem.INSTANCE,
                        SorterSubsystem.INSTANCE,
                        KickerSubsystem.INSTANCE,
                        ShooterSubsystem.INSTANCE,
                        VisionSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    @Override
    public void onInit() {
        sequencer = new ShotSequencer(RobotConfig.shotTiming(), SorterSubsystem.INSTANCE.getSlot());
        flywheelHeld = kickerHeld = fieldCentric = false;
        hoodTrim = 0;
        driverHeading = DriveSubsystem.INSTANCE.getPose().heading;
        telemetry.addLine("Ready. Alliance: " + (alliance == null ? "either goal" : alliance));
        telemetry.addData("Limelight", VisionSubsystem.INSTANCE.isConnected() ? "connected" : "NOT FOUND");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        bindMechanisms(Gamepads.gamepad1());
        bindMechanisms(Gamepads.gamepad2());
        GamepadEx g1 = Gamepads.gamepad1();
        g1.share().whenBecomesTrue(() -> fieldCentric = !fieldCentric);
        g1.options().whenBecomesTrue(() -> driverHeading = DriveSubsystem.INSTANCE.getPose().heading);
        lastNanos = System.nanoTime();
    }

    private void bindMechanisms(GamepadEx g) {
        g.triangle().whenBecomesTrue(IntakeSubsystem.INSTANCE.toggle);
        g.cross().whenBecomesTrue(IntakeSubsystem.INSTANCE.ejectOn);
        g.cross().whenBecomesFalse(IntakeSubsystem.INSTANCE.ejectOff);

        g.square().whenBecomesTrue(() -> manualSorter(SorterSubsystem.previous(SorterSubsystem.INSTANCE.getSlot())));
        g.circle().whenBecomesTrue(() -> manualSorter(SorterSubsystem.next(SorterSubsystem.INSTANCE.getSlot())));

        g.rightBumper().whenBecomesTrue(() -> flywheelHeld = true);
        g.rightBumper().whenBecomesFalse(() -> flywheelHeld = false);
        g.leftBumper().whenBecomesTrue(() -> kickerHeld = true);
        g.leftBumper().whenBecomesFalse(() -> kickerHeld = false);

        g.dpadUp().whenBecomesTrue(() -> hoodTrim += RobotConfig.HOOD_STEP);
        g.dpadDown().whenBecomesTrue(() -> hoodTrim -= RobotConfig.HOOD_STEP);
        g.dpadLeft().whenBecomesTrue(this::startVolley);
        g.dpadRight().whenBecomesTrue(() -> sequencer.cancel());
    }

    private void manualSorter(int slot) {
        if (sequencer.isBusy()) return; // the volley owns the sorter until it finishes or is cancelled
        SorterSubsystem.INSTANCE.setSlot(slot);
        sequencer.setSorterSlot(slot);
    }

    private void startVolley() {
        // No colour sensor: treat all three slots as loaded with unknown colours.
        ArtifactColor[] slots = {ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN, ArtifactColor.UNKNOWN};
        sequencer.setSorterSlot(SorterSubsystem.INSTANCE.getSlot());
        sequencer.start(FirePlanner.plan(slots, null, 0, SorterSubsystem.INSTANCE.getSlot()));
    }

    @Override
    public void onUpdate() {
        long now = System.nanoTime();
        double dt = MathUtil.clamp((now - lastNanos) / 1e9, 0, 0.1);
        loopMs = dt * 1000;
        lastNanos = now;

        DriveSubsystem drive = DriveSubsystem.INSTANCE;
        ShooterSubsystem shooter = ShooterSubsystem.INSTANCE;
        VisionSubsystem vision = VisionSubsystem.INSTANCE;
        vision.setRobotYawDegrees(Math.toDegrees(drive.getPose().heading));

        // ---- vision
        VisionSubsystem.GoalSighting goal = vision.getGoal(alliance);
        if (goal != null) {
            double d = FieldGeometry.floorDistanceFromRange(goal.rangeInches, RobotConfig.LIMELIGHT_HEIGHT_IN);
            if (MathUtil.isFinite(d)) lastGoalDistance = d;
            sinceGoalSeen = 0;
        } else {
            sinceGoalSeen += dt;
        }
        Motif seen = vision.getMotif();
        if (seen != null) motif = seen;
        boolean distanceFresh = sinceGoalSeen < 1.0 && MathUtil.isFinite(lastGoalDistance);

        // ---- drive
        Gamepad gp = gamepad1;
        double scale = RobotConfig.DRIVER_MAX_POWER * (gp.left_trigger > 0.3 ? RobotConfig.SLOW_MODE_SCALE : 1.0);
        double forward = -MathUtil.deadband(gp.left_stick_y, RobotConfig.STICK_DEADBAND) * scale;
        double left = -MathUtil.deadband(gp.left_stick_x, RobotConfig.STICK_DEADBAND) * scale * RobotConfig.STRAFE_MULTIPLIER;
        double turn = -MathUtil.deadband(gp.right_stick_x, RobotConfig.STICK_DEADBAND) * scale;

        boolean aiming = gp.right_trigger > 0.3;
        if (aiming && goal != null) {
            turn = aim.calculate(AimController.errorFromVision(goal.txDegrees), dt);
        } else {
            aim.reset();
        }

        if (fieldCentric) {
            double[] r = MecanumKinematics.fieldToRobot(forward, left, drive.getPose().heading - driverHeading);
            drive.drive(r[0], r[1], turn);
        } else {
            drive.drive(forward, left, turn);
        }

        // ---- shooter: distance-based when the goal was seen recently, competition shot otherwise
        ShotTable.Shot shot = distanceFresh
                ? shotTable.lookup(lastGoalDistance)
                : new ShotTable.Shot(Double.NaN, RobotConfig.FLYWHEEL_DEFAULT_TPS, RobotConfig.HOOD_DEFAULT);
        shooter.setHood(shot.hood + hoodTrim);
        boolean spin = flywheelHeld || sequencer.isBusy();
        shooter.setTargetVelocity(spin ? shot.flywheelTps : 0);

        // ---- sorter + kicker: volley owns them while it runs
        sequencer.update(dt, shooter.isReady());
        if (sequencer.isBusy()) {
            SorterSubsystem.INSTANCE.setSlot(sequencer.getSorterSlot());
            KickerSubsystem.INSTANCE.setFiring(sequencer.isKickerFiring());
        } else {
            KickerSubsystem.INSTANCE.setFiring(kickerHeld);
        }

        // ---- telemetry
        telemetry.addData("Drive", fieldCentric ? "FIELD-centric" : "robot-centric");
        telemetry.addData("Pose", drive.getPose());
        telemetry.addData("Flywheel", "%s target %.0f  actual %.0f  pwr %.2f%s",
                shooter.isReady() ? "READY" : "spinning", shooter.getTargetVelocity(), shooter.getVelocity(),
                shooter.getPower(), shooter.isEncoderFault() ? "  ENCODER FAULT (open loop)" : "");
        telemetry.addData("Hood", "%.2f (trim %+.2f)", shooter.getHood(), hoodTrim);
        telemetry.addData("Sorter slot", SorterSubsystem.INSTANCE.getSlot());
        telemetry.addData("Volley", sequencer.getState());
        telemetry.addData("Intake", IntakeSubsystem.INSTANCE.getMode());
        telemetry.addData("Goal", goal == null ? "not visible"
                : String.format(java.util.Locale.US, "tag %d tx %.1f° dist %.0fin", goal.tagId, goal.txDegrees, lastGoalDistance));
        telemetry.addData("Motif", motif == null ? "unknown" : motif);
        telemetry.addData("Battery", "%.1fV  loop %.0fms", shooter.getVoltage(), loopMs);
        telemetry.update();
    }

    @Override
    public void onStop() {
        VisionSubsystem.INSTANCE.stop();
    }
}
