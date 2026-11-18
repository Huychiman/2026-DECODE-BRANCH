package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.logic.Alliance;
import org.firstinspires.ftc.teamcode.logic.AutoPaths;

/** Shoot 3 preloads in MOTIF order, then LEAVE. Start pose: see {@link AutoPaths}. */
@Autonomous(name = "Auto Blue Audience", group = "Auto", preselectTeleOp = "TeleOp Blue")
public class AutoBlueAudience extends BaseAuto {
    public AutoBlueAudience() {
        super(Alliance.BLUE, AutoPaths.Start.AUDIENCE);
    }
}
