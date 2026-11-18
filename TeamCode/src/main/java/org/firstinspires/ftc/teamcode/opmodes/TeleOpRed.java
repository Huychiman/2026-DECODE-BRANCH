package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.logic.Alliance;

/** See {@link BaseTeleOp} for the control layout. */
@TeleOp(name = "TeleOp Red", group = "TeleOp")
public class TeleOpRed extends BaseTeleOp {
    public TeleOpRed() {
        super(Alliance.RED);
    }
}
