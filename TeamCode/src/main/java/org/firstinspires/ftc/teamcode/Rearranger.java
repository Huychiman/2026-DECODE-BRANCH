package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.ftc.ActiveOpMode;

public class Rearranger implements Subsystem {
    public static final Rearranger INSTANCE = new Rearranger();
    private Rearranger() {}

    private final ServoEx servo = new ServoEx("rearranger_servo");

    // Tune these later
    private static final double SLOT_1_POS = 0.10;
    private static final double SLOT_2_POS = 0.50;
    private static final double SLOT_3_POS = 0.90;

    private int slot = 1;
    private double targetPos = SLOT_1_POS;

    public final Command left = new LambdaCommand()
            .setStart(() -> {
                slot = (slot == 1) ? 3 : slot - 1;
                targetPos = slotToPos(slot);

                ActiveOpMode.telemetry().addData("Rearranger", "LEFT slot=%d pos=%.2f", slot, targetPos);
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Rearranger Left");

    public final Command right = new LambdaCommand()
            .setStart(() -> {
                slot = (slot == 3) ? 1 : slot + 1;
                targetPos = slotToPos(slot);

                ActiveOpMode.telemetry().addData("Rearranger", "RIGHT slot=%d pos=%.2f", slot, targetPos);
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Rearranger Right");

    @Override
    public void periodic() {
        servo.setPosition(targetPos);
        //long now = System.currentTimeMillis();
        //boolean high = (now / 1000) % 2 == 0; // flips every 1s
        //servo.setPosition(high ? 0.9 : 0.1);
    }


    private double slotToPos(int s) {
        switch (s) {
            case 1: return SLOT_1_POS;
            case 2: return SLOT_2_POS;
            case 3: return SLOT_3_POS;
            default: return SLOT_1_POS;
        }
    }

    public int getSlot() { return slot; }
    public double getTargetPos() { return targetPos; }


}
