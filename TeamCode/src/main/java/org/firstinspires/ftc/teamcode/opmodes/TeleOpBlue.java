package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.logic.Alliance;

/** See {@link BaseTeleOp} for the control layout. */
@TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOpBlue extends BaseTeleOp {
    public TeleOpBlue() {
        super(Alliance.BLUE);
    }
}
