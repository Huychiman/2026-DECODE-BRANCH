package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.FlywheelController;
import org.firstinspires.ftc.teamcode.logic.MathUtil;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/** Flywheel (closed-loop velocity, voltage compensated) + mirrored hood servos. */
public final class ShooterSubsystem implements Subsystem {
    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

    private CachedMotor flywheel;
    private CachedServo hoodLeft, hoodRight;
    private VoltageSensor battery;
    private FlywheelController controller;
    private double hood = RobotConfig.HOOD_DEFAULT;
    private double lastNanos;
    private double voltage = 12.5;

    private ShooterSubsystem() {}

    @Override
    public void initialize() {
        HardwareMap hw = ActiveOpMode.hardwareMap();
        flywheel = new CachedMotor(hw, RobotConfig.SHOOTER_MOTOR, false, false);
        hoodLeft = new CachedServo(hw, RobotConfig.HOOD_LEFT);
        hoodRight = new CachedServo(hw, RobotConfig.HOOD_RIGHT);
        battery = hw.voltageSensor.iterator().hasNext() ? hw.voltageSensor.iterator().next() : null;
        controller = new FlywheelController(RobotConfig.flywheelGains());
        hood = RobotConfig.HOOD_DEFAULT;
        lastNanos = System.nanoTime();
    }

    public void setTargetVelocity(double ticksPerSecond) {
        controller.setTarget(ticksPerSecond);
    }

    public void stop() {
        controller.setTarget(0);
    }

    public double getTargetVelocity() {
        return controller.getTarget();
    }

    public double getVelocity() {
        return controller.getMeasured();
    }

    public boolean isReady() {
        return controller.isReady();
    }

    public boolean isEncoderFault() {
        return controller.isEncoderFault();
    }

    public double getPower() {
        return controller.getPower();
    }

    public double getVoltage() {
        return voltage;
    }

    public void setHood(double position) {
        hood = MathUtil.clamp(position, RobotConfig.HOOD_MIN, RobotConfig.HOOD_MAX);
    }

    public double getHood() {
        return hood;
    }

    @Override
    public void periodic() {
        double now = System.nanoTime();
        double dt = MathUtil.clamp((now - lastNanos) / 1e9, 0, 0.1);
        lastNanos = now;
        if (battery != null) {
            double v = battery.getVoltage();
            if (v > 6) voltage = v;
        }
        // abs(): the wheel only spins one way, so a reversed encoder polarity cannot cause runaway.
        flywheel.setPower(controller.update(Math.abs(flywheel.getVelocity()), voltage, dt));
        hoodLeft.setPosition(hood);
        hoodRight.setPosition(1.0 - hood);
    }
}
