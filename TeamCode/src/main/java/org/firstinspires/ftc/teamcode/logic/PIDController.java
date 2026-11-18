package org.firstinspires.ftc.teamcode.logic;

/** Minimal PID with explicit dt (so it is deterministic in tests) and integral clamping. */
public final class PIDController {
    private final double kP, kI, kD, integralLimit;
    private double integral;
    private double lastError;
    private boolean hasLast;

    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 1.0);
    }

    public PIDController(double kP, double kI, double kD, double integralLimit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralLimit = integralLimit;
    }

    public double calculate(double error, double dt) {
        if (dt <= 0 || !MathUtil.isFinite(error)) return kP * (MathUtil.isFinite(error) ? error : 0);
        integral = MathUtil.clamp(integral + error * dt, -integralLimit, integralLimit);
        double derivative = hasLast ? (error - lastError) / dt : 0.0;
        lastError = error;
        hasLast = true;
        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        hasLast = false;
    }
}
