package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.CachedMotor;

/**
 * Checks wiring, directions and encoders, and measures DRIVE_INCHES_PER_TICK.
 *
 * <p>Hold a face button to spin one wheel at 30% power. EVERY wheel must spin so it would push the
 * robot FORWARD and its encoder must count UP. If not, flip LEFT_REVERSED / RIGHT_REVERSED or swap
 * the port. Then press Share to zero, push the robot exactly 48in forward by hand, and copy the
 * "inches per tick" value into RobotConfig.DRIVE_INCHES_PER_TICK.
 */
@TeleOp(name = "Tune: Drive Motors", group = "Tune")
public class DriveMotorTest extends OpMode {
    private CachedMotor fl, fr, bl, br;
    private final double[] zero = new double[4];
    private boolean lastShare;

    @Override
    public void init() {
        fl = new CachedMotor(hardwareMap, RobotConfig.LEFT_FRONT, RobotConfig.LEFT_REVERSED, true);
        fr = new CachedMotor(hardwareMap, RobotConfig.RIGHT_FRONT, RobotConfig.RIGHT_REVERSED, true);
        bl = new CachedMotor(hardwareMap, RobotConfig.LEFT_BACK, RobotConfig.LEFT_REVERSED, true);
        br = new CachedMotor(hardwareMap, RobotConfig.RIGHT_BACK, RobotConfig.RIGHT_REVERSED, true);
    }

    @Override
    public void loop() {
        fl.setPower(gamepad1.square ? 0.3 : 0);
        fr.setPower(gamepad1.triangle ? 0.3 : 0);
        bl.setPower(gamepad1.cross ? 0.3 : 0);
        br.setPower(gamepad1.circle ? 0.3 : 0);

        double[] t = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
        if (gamepad1.share && !lastShare) System.arraycopy(t, 0, zero, 0, 4);
        lastShare = gamepad1.share;
        double avg = 0;
        for (int i = 0; i < 4; i++) avg += (t[i] - zero[i]) / 4.0;

        telemetry.addLine("Square=FL  Triangle=FR  Cross=BL  Circle=BR (30% power)");
        telemetry.addLine("Each wheel must push FORWARD and its count must go UP.");
        telemetry.addData("FL", "%.0f", t[0] - zero[0]);
        telemetry.addData("FR", "%.0f", t[1] - zero[1]);
        telemetry.addData("BL", "%.0f", t[2] - zero[2]);
        telemetry.addData("BR", "%.0f", t[3] - zero[3]);
        telemetry.addData("Average ticks since Share", "%.0f", avg);
        telemetry.addData("inches per tick if pushed 48in", avg != 0 ? String.format(java.util.Locale.US, "%.6f", 48.0 / avg) : "-");
        telemetry.addData("current config value", "%.6f", RobotConfig.DRIVE_INCHES_PER_TICK);
    }
}
