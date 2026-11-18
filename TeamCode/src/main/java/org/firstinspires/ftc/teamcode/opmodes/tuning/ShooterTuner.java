package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.FieldGeometry;
import org.firstinspires.ftc.teamcode.logic.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.CachedMotor;

/**
 * Tunes the flywheel and builds the ShotTable.
 *
 * <p>1) kV: hold Square (open loop, 0.8 power) until speed settles; copy "kV estimate" into
 * RobotConfig.flywheelGains(). 2) Shot table: park at a distance, hold RB (closed loop), adjust
 * speed (D-pad up/down) and hood (D-pad left/right) until LB-kicked shots go in, then write down the
 * "ROW" line as a {@code .add(distance, tps, hood)} entry in RobotConfig.shotTable(). Repeat for 4-6
 * distances across both launch zones.
 */
@TeleOp(name = "Tune: Shooter", group = "Tune")
public class ShooterTuner extends OpMode {
    private CachedMotor flywheel;
    private Servo hoodL, hoodR, kicker;
    private VoltageSensor battery;
    private Limelight3A limelight;
    private FlywheelController controller;
    private double target = RobotConfig.FLYWHEEL_DEFAULT_TPS;
    private double hood = RobotConfig.HOOD_DEFAULT;
    private boolean up, down, left, right;
    private long last;

    @Override
    public void init() {
        flywheel = new CachedMotor(hardwareMap, RobotConfig.SHOOTER_MOTOR, false, false);
        hoodL = hardwareMap.get(Servo.class, RobotConfig.HOOD_LEFT);
        hoodR = hardwareMap.get(Servo.class, RobotConfig.HOOD_RIGHT);
        kicker = hardwareMap.get(Servo.class, RobotConfig.KICKER_SERVO);
        battery = hardwareMap.voltageSensor.iterator().next();
        limelight = hardwareMap.tryGet(Limelight3A.class, RobotConfig.LIMELIGHT);
        if (limelight != null) {
            limelight.pipelineSwitch(RobotConfig.LIMELIGHT_APRILTAG_PIPELINE);
            limelight.start();
        }
        controller = new FlywheelController(RobotConfig.flywheelGains());
        last = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = Math.min(0.1, (now - last) / 1e9);
        last = now;

        if (gamepad1.dpad_up && !up) target += RobotConfig.FLYWHEEL_STEP_TPS;
        if (gamepad1.dpad_down && !down) target = Math.max(0, target - RobotConfig.FLYWHEEL_STEP_TPS);
        if (gamepad1.dpad_right && !right) hood = Math.min(RobotConfig.HOOD_MAX, hood + 0.02);
        if (gamepad1.dpad_left && !left) hood = Math.max(RobotConfig.HOOD_MIN, hood - 0.02);
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        left = gamepad1.dpad_left;
        right = gamepad1.dpad_right;

        double velocity = Math.abs(flywheel.getVelocity());
        double volts = battery.getVoltage();
        double power;
        if (gamepad1.square) {
            power = 0.8;
            controller.setTarget(0);
        } else {
            controller.setTarget(gamepad1.right_bumper ? target : 0);
            power = controller.update(velocity, volts, dt);
        }
        flywheel.setPower(power);
        hoodL.setPosition(hood);
        hoodR.setPosition(1.0 - hood);
        kicker.setPosition(gamepad1.left_bumper ? RobotConfig.KICKER_FIRE : RobotConfig.KICKER_REST);

        double distance = Double.NaN;
        if (limelight != null) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                for (LLResultTypes.FiducialResult t : r.getFiducialResults()) {
                    if (t.getFiducialId() == 20 || t.getFiducialId() == 24) {
                        Position p = t.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                        double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                        distance = FieldGeometry.floorDistanceFromRange(range, RobotConfig.LIMELIGHT_HEIGHT_IN);
                    }
                }
            }
        }

        telemetry.addLine("Square: open-loop 0.8 | RB: closed-loop target | LB: kick");
        telemetry.addLine("D-pad up/down: speed | D-pad left/right: hood");
        telemetry.addData("Target", "%.0f ticks/s (%.0f rpm)", target, target / RobotConfig.FLYWHEEL_TICKS_PER_REV * 60);
        telemetry.addData("Measured", "%.0f ticks/s", velocity);
        telemetry.addData("Power / battery", "%.2f / %.2fV", power, volts);
        if (velocity > 100 && power > 0.1) {
            telemetry.addData("kV estimate (steady state)", "%.7f", power * (volts / 12.5) / velocity);
        }
        telemetry.addData("Ready", controller.isReady());
        telemetry.addData("Hood", "%.2f", hood);
        telemetry.addData("Goal floor distance", Double.isNaN(distance) ? "no goal tag" : String.format(java.util.Locale.US, "%.1f in", distance));
        telemetry.addData("ROW", ".add(%.0f, %.0f, %.2f)", Double.isNaN(distance) ? 0.0 : distance, target, hood);
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
