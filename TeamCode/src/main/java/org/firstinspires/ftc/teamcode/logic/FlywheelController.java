package org.firstinspires.ftc.teamcode.logic;

/**
 * Velocity controller for the shooter flywheel: voltage-compensated feedforward + proportional.
 *
 * <p>Why: the old code ran the flywheel at a fixed 0.8 power, so shot distance drifted as the battery
 * drained. Feedforward {@code kV * target} reproduces that 0.8 power at a full battery, the voltage
 * term keeps the wheel speed the same as the battery sags, and kP closes the remaining error.
 *
 * <p>Encoder safety: if the wheel is powered but the encoder reports ~0 for a while (cable unplugged)
 * the controller latches {@link #isEncoderFault()} and falls back to pure feedforward, and
 * {@link #isReady()} switches to a spin-up timer so the robot still shoots.
 */
public final class FlywheelController {
    public static final class Gains {
        public double kV = 1.0 / 2800.0; // power per tick/s at nominal voltage (6000rpm, 28 cpr)
        public double kS = 0.0; // static friction power
        public double kP = 0.0008; // power per tick/s of error
        public double nominalVoltage = 12.5;
        public double readyTolerance = 60; // ticks/s
        public double readySettleSeconds = 0.12;
        public double faultPowerThreshold = 0.4;
        public double faultSeconds = 1.0;
        public double faultVelocityThreshold = 40; // ticks/s
        public double openLoopSpinUpSeconds = 1.2;
    }

    private final Gains g;
    private double target;
    private double measured;
    private double power;
    private double inToleranceFor;
    private double faultTimer;
    private double spinUpTimer;
    private boolean encoderFault;

    public FlywheelController(Gains gains) {
        this.g = gains;
    }

    public void setTarget(double ticksPerSecond) {
        double t = Math.max(0, ticksPerSecond);
        if (Math.abs(t - target) > g.readyTolerance) {
            inToleranceFor = 0;
            spinUpTimer = 0;
        }
        target = t;
    }

    public double getTarget() {
        return target;
    }

    /** @return motor power in [0, 1] */
    public double update(double measuredTicksPerSecond, double batteryVoltage, double dt) {
        measured = measuredTicksPerSecond;
        if (target <= 0) {
            power = 0;
            inToleranceFor = 0;
            spinUpTimer = 0;
            return power;
        }
        double v = (batteryVoltage > 6.0 && MathUtil.isFinite(batteryVoltage)) ? batteryVoltage : g.nominalVoltage;
        double ff = (g.kV * target + g.kS) * (g.nominalVoltage / v);
        double fb = encoderFault ? 0 : g.kP * (target - measured);
        power = MathUtil.clamp(ff + fb, 0, 1);

        if (!encoderFault) {
            if (power >= g.faultPowerThreshold && Math.abs(measured) < g.faultVelocityThreshold) {
                faultTimer += dt;
                if (faultTimer >= g.faultSeconds) encoderFault = true;
            } else {
                faultTimer = 0;
            }
        }
        if (Math.abs(target - measured) <= g.readyTolerance) inToleranceFor += dt;
        else inToleranceFor = 0;
        spinUpTimer += dt;
        return power;
    }

    public boolean isReady() {
        if (target <= 0) return false;
        if (encoderFault) return spinUpTimer >= g.openLoopSpinUpSeconds;
        return inToleranceFor >= g.readySettleSeconds;
    }

    public boolean isEncoderFault() {
        return encoderFault;
    }

    public double getPower() {
        return power;
    }

    public double getMeasured() {
        return measured;
    }

    public void reset() {
        target = 0;
        power = 0;
        inToleranceFor = 0;
        faultTimer = 0;
        spinUpTimer = 0;
        encoderFault = false;
    }
}
