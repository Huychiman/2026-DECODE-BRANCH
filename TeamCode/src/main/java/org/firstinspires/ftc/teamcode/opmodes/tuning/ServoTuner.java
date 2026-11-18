package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Finds servo positions for RobotConfig (sorter slots, kicker rest/fire, hood range).
 *
 * <p>Bumpers pick the servo, D-pad up/down nudges +/-0.01, triangle/cross nudge +/-0.05. The hood
 * entry moves both hood servos mirrored, exactly like the shooter does.
 */
@TeleOp(name = "Tune: Servos", group = "Tune")
public class ServoTuner extends OpMode {
    private static final String[] NAMES = {
        RobotConfig.SORTER_SERVO, RobotConfig.KICKER_SERVO, "hood (" + RobotConfig.HOOD_LEFT + " + mirrored " + RobotConfig.HOOD_RIGHT + ")"
    };
    private Servo sorter, kicker, hoodL, hoodR;
    private final double[] pos = {RobotConfig.SORTER_POSITIONS[0], RobotConfig.KICKER_REST, RobotConfig.HOOD_DEFAULT};
    private int selected;
    private boolean lb, rb, up, down, tri, cross;

    @Override
    public void init() {
        sorter = hardwareMap.get(Servo.class, RobotConfig.SORTER_SERVO);
        kicker = hardwareMap.get(Servo.class, RobotConfig.KICKER_SERVO);
        hoodL = hardwareMap.get(Servo.class, RobotConfig.HOOD_LEFT);
        hoodR = hardwareMap.get(Servo.class, RobotConfig.HOOD_RIGHT);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper && !lb) selected = (selected + NAMES.length - 1) % NAMES.length;
        if (gamepad1.right_bumper && !rb) selected = (selected + 1) % NAMES.length;
        double step = 0;
        if (gamepad1.dpad_up && !up) step = 0.01;
        if (gamepad1.dpad_down && !down) step = -0.01;
        if (gamepad1.triangle && !tri) step = 0.05;
        if (gamepad1.cross && !cross) step = -0.05;
        lb = gamepad1.left_bumper;
        rb = gamepad1.right_bumper;
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        tri = gamepad1.triangle;
        cross = gamepad1.cross;
        pos[selected] = Math.max(0, Math.min(1, pos[selected] + step));

        sorter.setPosition(pos[0]);
        kicker.setPosition(pos[1]);
        hoodL.setPosition(pos[2]);
        hoodR.setPosition(1.0 - pos[2]);

        telemetry.addLine("Bumpers: choose servo | D-pad: +/-0.01 | Triangle/Cross: +/-0.05");
        for (int i = 0; i < NAMES.length; i++) {
            telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.2f", pos[i]);
        }
    }
}
