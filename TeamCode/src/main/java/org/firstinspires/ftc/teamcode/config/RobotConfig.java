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

    /** TUNE: goBILDA 312rpm (537.7 ticks/rev) with 104mm wheels. Use "Tune: Drive Motors". */
    public static final double DRIVE_INCHES_PER_TICK = (104.0 / 25.4 * Math.PI) / 537.7;
    /**
     * TUNE: actual strafe distance / odometry strafe distance (strafe 48in by hand). Mecanum rollers
     * slip sideways, so the real value is usually a bit below 1.0.
     */
    public static final double LATERAL_MULTIPLIER = 1.0;

    /** Robot bumper footprint in inches (18in max sizing cube). TUNE to the real robot. */
    public static final double ROBOT_LENGTH = 18.0;
    public static final double ROBOT_WIDTH = 18.0;

    // ---------------------------------------------------------------- sorter (rearranger)
    /** Servo positions for slots 0, 1, 2 (were "slot 1..3" in the competition code). */
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
    /** Flywheel motor encoder ticks per revolution (goBILDA 6000rpm bare motor = 28). TUNE. */
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
    /** TUNE: Limelight lens height above the floor, inches. Converts tag range to floor distance. */
    public static final double LIMELIGHT_HEIGHT_IN = 10.0;

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
        return new AimController.Gains();
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
