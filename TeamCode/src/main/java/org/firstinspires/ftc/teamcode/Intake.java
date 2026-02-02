package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private MotorEx motor = new MotorEx("intake_motor");

    private boolean running = false;
    private double power = 0.0;

    public static final double INTAKE_POWER = 1.0;
    public static final double STOP_POWER = 0.0;

    public final Command toggle = new LambdaCommand()
            .setStart(() -> {
                running = !running;
                power = running ? INTAKE_POWER : STOP_POWER;
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Intake toggle");

    @Override
    public void periodic(){
        motor.setPower(power);
    }



}
