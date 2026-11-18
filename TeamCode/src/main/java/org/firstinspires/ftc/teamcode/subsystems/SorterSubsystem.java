package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/** The 3-slot "rearranger" servo that lines a stored artifact up with the kicker. */
public final class SorterSubsystem implements Subsystem {
    public static final SorterSubsystem INSTANCE = new SorterSubsystem();

    private CachedServo servo;
    private int slot = RobotConfig.SORTER_START_SLOT;

    private SorterSubsystem() {}

    @Override
    public void initialize() {
        servo = new CachedServo(ActiveOpMode.hardwareMap(), RobotConfig.SORTER_SERVO);
        slot = RobotConfig.SORTER_START_SLOT;
    }

    public static int slotCount() {
        return RobotConfig.SORTER_POSITIONS.length;
    }

    /** Previous slot, wrapping 0 -> 2 (competition "left" button). */
    public static int previous(int s) {
        return (s + slotCount() - 1) % slotCount();
    }

    /** Next slot, wrapping 2 -> 0 (competition "right" button). */
    public static int next(int s) {
        return (s + 1) % slotCount();
    }

    public void setSlot(int s) {
        slot = ((s % slotCount()) + slotCount()) % slotCount();
    }

    public int getSlot() {
        return slot;
    }

    @Override
    public void periodic() {
        servo.setPosition(RobotConfig.SORTER_POSITIONS[slot]);
    }
}
