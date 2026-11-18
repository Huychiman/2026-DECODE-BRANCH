package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.logic.Alliance;
import org.firstinspires.ftc.teamcode.logic.AutoPaths;

/** Shoot 3 preloads in MOTIF order, then LEAVE. Start pose: see {@link AutoPaths}. */
@Autonomous(name = "Auto Red Goal", group = "Auto", preselectTeleOp = "TeleOp Red")
public class AutoRedGoal extends BaseAuto {
    public AutoRedGoal() {
        super(Alliance.RED, AutoPaths.Start.GOAL);
    }
}
