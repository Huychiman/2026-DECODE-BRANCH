package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * The "picker" servo that pushes the lined-up artifact into the flywheel. This is now the ONLY code
 * that writes to pickerServo (the competition code had two subsystems fighting over it).
 */
public final class KickerSubsystem implements Subsystem {
    public static final KickerSubsystem INSTANCE = new KickerSubsystem();

    private CachedServo servo;
    private boolean firing;

    private KickerSubsystem() {}

    @Override
    public void initialize() {
        servo = new CachedServo(ActiveOpMode.hardwareMap(), RobotConfig.KICKER_SERVO);
        firing = false;
    }

    public void setFiring(boolean f) {
        firing = f;
    }

    public boolean isFiring() {
        return firing;
    }

    @Override
    public void periodic() {
        servo.setPosition(firing ? RobotConfig.KICKER_FIRE : RobotConfig.KICKER_REST);
    }
}
