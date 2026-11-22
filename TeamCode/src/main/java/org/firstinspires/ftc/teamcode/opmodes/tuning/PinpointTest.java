package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Checks the goBILDA Pinpoint and its two dead-wheel pods. Push the robot by hand (motors are not used):
 * <ol>
 *   <li>Push straight forward 48in: X must read about +48. If it reads -48, flip PINPOINT_X_REVERSED.</li>
 *   <li>Push straight left 48in: Y must read about +48. If it reads -48, flip PINPOINT_Y_REVERSED.</li>
 *   <li>Spin in place 10 turns and return to the start: X and Y must stay within ~1in. If they drift in a
 *       circle, an offset sign or size is wrong (PINPOINT_X/Y_POD_OFFSET_MM).</li>
 * </ol>
 * Press A to zero again (robot still).
 */
@TeleOp(name = "Tune: Pinpoint", group = "Tune")
public class PinpointTest extends OpMode {
    private GoBildaPinpointDriver pinpoint;
    private boolean prevA;

    @Override
    public void init() {
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
        if (pinpoint == null) return;
        pinpoint.setOffsets(RobotConfig.PINPOINT_X_POD_OFFSET_MM, RobotConfig.PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                RobotConfig.PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                RobotConfig.PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        if (pinpoint == null) {
            telemetry.addLine("No I2C device named \"" + RobotConfig.PINPOINT + "\" in the configuration.");
            telemetry.addLine("Add it as \"goBILDA Pinpoint Odometry Computer\" on the hub's I2C port.");
            return;
        }
        if (gamepad1.a && !prevA) pinpoint.resetPosAndIMU();
        prevA = gamepad1.a;

        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("X (forward +)", "%.2f in", p.getX(DistanceUnit.INCH));
        telemetry.addData("Y (left +)", "%.2f in", p.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (CCW +)", "%.1f deg", p.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Raw encoders X / Y", "%d / %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
        telemetry.addData("Offsets X / Y", "%.1f / %.1f mm", RobotConfig.PINPOINT_X_POD_OFFSET_MM,
                RobotConfig.PINPOINT_Y_POD_OFFSET_MM);
        telemetry.addData("Update rate", "%.0f Hz", pinpoint.getFrequency());
        telemetry.addLine("Push 48in forward -> X +48; 48in left -> Y +48; spin 10x -> X, Y stay near 0. A = zero.");
    }
}
