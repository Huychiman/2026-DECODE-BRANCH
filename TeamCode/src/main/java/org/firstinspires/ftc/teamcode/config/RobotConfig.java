package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.logic.AimController;
import org.firstinspires.ftc.teamcode.logic.ArtifactColor;
import org.firstinspires.ftc.teamcode.logic.AutoRoutine;
import org.firstinspires.ftc.teamcode.logic.DriveToPose;
import org.firstinspires.ftc.teamcode.logic.FlywheelController;
import org.firstinspires.ftc.teamcode.logic.ShotSequencer;
import org.firstinspires.ftc.teamcode.logic.ShotTable;

/**
 * Every hardware name and tunable number in one place.
 *
 * <p>Hardware names match the robot's existing Driver Station configuration (taken from the code
 * that ran at the first competition), so no reconfiguration is needed.
 *
 * <p>Values marked TUNE are reasonable defaults that must be checked on the real robot with the
 * OpModes in the "Tune" group. Everything else was carried over from the competition code.
 */
public final class RobotConfig {
    private RobotConfig() {}

    // ---------------------------------------------------------------- hardware names
    public static final String LEFT_FRONT = "leftFront";
    public static final String RIGHT_FRONT = "rightFront";
    public static final String LEFT_BACK = "leftBack";
    public static final String RIGHT_BACK = "rightBack";
    public static final String IMU = "imu";
    public static final String INTAKE = "intake_motor";
    public static final String SORTER_SERVO = "rearranger_servo";
    public static final String SHOOTER_MOTOR = "shooterMotor";
    public static final String HOOD_LEFT = "shooterAngle2"; // gets hood position
    public static final String HOOD_RIGHT = "shooterAngle1"; // gets 1 - hood position (mirrored)
    public static final String KICKER_SERVO = "pickerServo";
    public static final String LIMELIGHT = "limelight";
    /**
     * goBILDA Pinpoint odometry computer (I2C device type "goBILDA Pinpoint Odometry Computer"). The CAD
     * has one with two 4-bar dead-wheel pods. Optional: if it is not in the Driver Station configuration
     * the drive falls back to drive-motor encoders + the Control Hub IMU.
     */
    public static final String PINPOINT = "pinpoint";

    // ---------------------------------------------------------------- drive
    /*
     * Motor directions. The competition TeleOp reversed the LEFT motors and fed un-negated stick Y into
     * NextFTC's mecanum command, which is exactly the standard mecanum mix with every wheel's sign
     * flipped. Reversing the RIGHT motors instead is the same physical behaviour with
     * "positive power = drive forward" — MecanumDirectionTest proves the wheel outputs are identical.
     */
    public static final boolean LEFT_REVERSED = false;
    public static final boolean RIGHT_REVERSED = true;

    /** The competition code multiplied strafe by 1.1 (NextFTC default) to counter mecanum slip. */
    public static final double STRAFE_MULTIPLIER = 1.1;
    /** The competition code scaled every stick by 0.9. */
    public static final double DRIVER_MAX_POWER = 0.9;
    public static final double SLOW_MODE_SCALE = 0.4;
    public static final double STICK_DEADBAND = 0.05;

    /*
     * Drivetrain from the "Robot" CAD (Fusion project BFC 2025-2026, Chassis):
     * REV HD Hex motor (28 counts/motor rev) + UltraPlanetary 4:1 cartridge (REV-41-1602, real 76:21)
     * + 5:1 cartridge (REV-41-1603, real 68:13), then a 22T:22T HTD 5M belt (1:1) to a 104mm mecanum wheel.
     */
    public static final double DRIVE_WHEEL_DIAMETER_IN = 104.0 / 25.4;
    public static final double DRIVE_GEAR_RATIO = (76.0 / 21.0) * (68.0 / 13.0); // 18.93:1 (nominal 20:1)
    public static final double DRIVE_TICKS_PER_WHEEL_REV = 28.0 * DRIVE_GEAR_RATIO; // 530.05
    /** Check with "Tune: Drive Motors" (push 48in) if wheels or gearboxes change. */
    public static final double DRIVE_INCHES_PER_TICK = DRIVE_WHEEL_DIAMETER_IN * Math.PI / DRIVE_TICKS_PER_WHEEL_REV;
    /** Free speed at the wheel, in/s (HD Hex 6000rpm). The simulation uses 80% of this as loaded speed. */
    public static final double DRIVE_FREE_SPEED_IN_PER_S =
            6000.0 / 60.0 / DRIVE_GEAR_RATIO * DRIVE_WHEEL_DIAMETER_IN * Math.PI;
    /** CAD wheel centres: track width 39.2cm, wheelbase 29.7cm. */
    public static final double TRACK_WIDTH_IN = 39.2 / 2.54;
    public static final double WHEELBASE_IN = 29.7 / 2.54;
    /**
     * TUNE: actual strafe distance / odometry strafe distance (strafe 48in by hand). Mecanum rollers
     * slip sideways, so the real value is usually a bit below 1.0.
     */
    public static final double LATERAL_MULTIPLIER = 1.0;

    /*
     * Footprint from the CAD bounding box. Width (side plate to side plate) is 45.7cm = 18.0in. Length is
     * 48.1cm = 18.9in: the rigid chassis is 43cm, the cable channel sticks out 1.8cm at the back and the
     * intake roller 3.3cm at the front. NOTE: 18.9in is over the 18in starting size limit unless the
     * intake roller compresses or the real robot differs from the CAD. Measure it before inspection.
     * Rule checks and the simulation use the CAD value (conservative for walls and LEAVE).
     */
    public static final double ROBOT_LENGTH = 48.1 / 2.54;
    public static final double ROBOT_WIDTH = 45.7 / 2.54;

    // ---------------------------------------------------------------- Pinpoint odometry
    /*
     * Pod positions from the CAD, relative to the robot centre (middle of the four wheels). Pinpoint sign
     * convention: X (forward-measuring) pod offset is + to the LEFT, Y (strafe) pod offset is + FORWARD.
     * The forward pod is beside the right side plate, 19.45cm right of centre; the strafe pod is on the
     * left side, 1.45cm behind centre.
     * Verify with "Tune: Pinpoint": spin in place 10 turns, X and Y must stay within about 1in.
     */
    public static final double PINPOINT_X_POD_OFFSET_MM = -194.5;
    public static final double PINPOINT_Y_POD_OFFSET_MM = -14.5;
    /** TUNE with "Tune: Pinpoint": pushing forward must increase X, pushing left must increase Y. */
    public static final boolean PINPOINT_X_REVERSED = false;
    public static final boolean PINPOINT_Y_REVERSED = false;

    // ---------------------------------------------------------------- sorter (rearranger)
    /**
     * Servo positions for slots 0, 1, 2 (were "slot 1..3" in the competition code, tuned on the robot).
     * CAD check: the indexer is a 38T gear driving a 20T gear on the 3-cup carousel (120 degrees per
     * slot). With a 270-degree REV Smart Servo on the 38T gear that would be 0.234 per slot, not 0.40.
     * The competition values are kept because they were set on the real robot; if the cups do not line
     * up with the pusher, re-measure them with "Tune: Servos".
     */
    public static final double[] SORTER_POSITIONS = {0.10, 0.50, 0.90};
    public static final int SORTER_START_SLOT = 0;

    // ---------------------------------------------------------------- kicker (picker)
    /*
     * In the competition code the picker rested at 0.55 and went to 0.20 while LB was held.
     * (Shooter.java also wrote 0.20 to the same servo every loop — a conflict this rewrite removes.)
     */
    public static final double KICKER_REST = 0.55;
    public static final double KICKER_FIRE = 0.20;

    // ---------------------------------------------------------------- intake
    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = -0.6;

    // ---------------------------------------------------------------- shooter
    public static final double HOOD_DEFAULT = 0.50;
    public static final double HOOD_MIN = 0.10;
    public static final double HOOD_MAX = 0.90;
    public static final double HOOD_STEP = 0.05;
    public static final double FLYWHEEL_STEP_TPS = 50;
    /*
     * Shooter from the CAD: REV HD Hex motor -> 30T:30T GT2 belt -> four REV 90mm grip wheels. Hood: REV
     * Smart Servo with a 24T gear on a 264T sector, so the full servo range moves the hood ~24.5 degrees.
     * Speeds are MOTOR encoder ticks/s (28 per motor rev, 2800 ticks/s at the 6000rpm free speed).
     */
    /** REV HD Hex motor encoder counts per motor revolution. */
    public static final double FLYWHEEL_TICKS_PER_REV = 28;
    /** Old fixed power 0.8 x 2800 ticks/s free speed: the shot that worked at competition. */
    public static final double FLYWHEEL_DEFAULT_TPS = 2240;

    public static FlywheelController.Gains flywheelGains() {
        FlywheelController.Gains g = new FlywheelController.Gains();
        g.kV = 1.0 / 2800.0; // TUNE with "Tune: Shooter" (kV = power / measured steady speed)
        g.kS = 0.0;
        g.kP = 0.0008;
        g.nominalVoltage = 12.5;
        return g;
    }

    /**
     * TUNE: distance to goal tag (inches) -> flywheel ticks/s and hood. Until real points are recorded
     * every distance uses the competition shot (0.8 power-equivalent, hood 0.5), so behaviour is no
     * worse than before. Record real points with "Tune: Shooter" and replace these rows.
     */
    public static ShotTable shotTable() {
        return new ShotTable()
                .add(24, FLYWHEEL_DEFAULT_TPS, HOOD_DEFAULT)
                .add(144, FLYWHEEL_DEFAULT_TPS, HOOD_DEFAULT);
    }

    // ---------------------------------------------------------------- vision
    /** Limelight pipeline used at competition for AprilTags. */
    public static final int LIMELIGHT_APRILTAG_PIPELINE = 8;
    /*
     * Limelight 3A mount from the CAD (Shooter / camera holder). It sits 3.7in RIGHT of the shooter's
     * centre line, so "tx = 0" alone would aim the shooter 3.7in right of the tag. AimController corrects
     * this parallax using the distance to the goal.
     */
    /** Lens height above the floor, inches. Converts tag range to floor distance. */
    public static final double LIMELIGHT_HEIGHT_IN = 22.2 / 2.54;
    /** Lens position relative to the robot centre, inches (forward +, left +). */
    public static final double LIMELIGHT_FORWARD_IN = 19.5 / 2.54;
    public static final double LIMELIGHT_LEFT_IN = -9.3 / 2.54;

    // ---------------------------------------------------------------- autonomous
    /**
     * Colours loaded into sorter slots 0, 1, 2 before AUTO. Load the robot this way and the auto fires
     * them in the order the OBELISK motif asks for. Set to UNKNOWN to just fire 0, 1, 2.
     */
    public static final ArtifactColor[] PRELOAD = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};

    public static ShotSequencer.Timing shotTiming() {
        return new ShotSequencer.Timing();
    }

    public static AimController.Gains aimGains() {
        AimController.Gains g = new AimController.Gains();
        g.cameraForward = LIMELIGHT_FORWARD_IN;
        g.cameraLeft = LIMELIGHT_LEFT_IN;
        return g;
    }

    public static AutoRoutine.Settings autoSettings() {
        AutoRoutine.Settings s = new AutoRoutine.Settings();
        s.drive = new DriveToPose.Gains();
        s.aim = aimGains();
        s.shotTiming = shotTiming();
        s.shotTable = shotTable();
        return s;
    }
}
