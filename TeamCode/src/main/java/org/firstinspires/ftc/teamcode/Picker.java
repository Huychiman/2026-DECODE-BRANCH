package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class Picker implements Subsystem {
    public static final Picker INSTANCE = new Picker();
    private Picker() {}

    private final ServoEx picker = new ServoEx("pickerServo");

    private static final double PICKER_DOWN = 0.20;
    private static final double PICKER_UP   = 0.55;

    private double pickerTarget = PICKER_UP; // start UP (change if you want)

    public final Command pickerUp = new LambdaCommand()
            .setStart(() -> {
                pickerTarget = PICKER_UP;
                ActiveOpMode.telemetry().addData("Picker", "UP %.2f", PICKER_UP);
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Picker Up");

    public final Command pickerDown = new LambdaCommand()
            .setStart(() -> {
                pickerTarget = PICKER_DOWN;
                ActiveOpMode.telemetry().addData("Picker", "DOWN %.2f", PICKER_DOWN);
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Picker Down");

    @Override
    public void periodic() {
        picker.setPosition(pickerTarget);
    }
}
