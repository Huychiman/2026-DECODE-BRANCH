package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/** Intake roller. Toggle on/off (competition behaviour) plus a hold-to-eject override. */
public final class IntakeSubsystem implements Subsystem {
    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();

    public enum Mode {
        OFF,
        IN,
        OUT
    }

    private CachedMotor motor;
    private Mode mode = Mode.OFF;
    private boolean ejecting;

    private IntakeSubsystem() {}

    @Override
    public void initialize() {
        motor = new CachedMotor(ActiveOpMode.hardwareMap(), RobotConfig.INTAKE, false, false);
        mode = Mode.OFF;
        ejecting = false;
    }

    public final Command toggle = new InstantCommand("Intake toggle", () -> mode = (mode == Mode.IN) ? Mode.OFF : Mode.IN);
    public final Command ejectOn = new InstantCommand("Intake eject on", () -> ejecting = true);
    public final Command ejectOff = new InstantCommand("Intake eject off", () -> ejecting = false);

    public void setMode(Mode m) {
        mode = m;
    }

    public Mode getMode() {
        return ejecting ? Mode.OUT : mode;
    }

    @Override
    public void periodic() {
        switch (getMode()) {
            case IN:
                motor.setPower(RobotConfig.INTAKE_POWER);
                break;
            case OUT:
                motor.setPower(RobotConfig.OUTTAKE_POWER);
                break;
            default:
                motor.setPower(0);
        }
    }
}
