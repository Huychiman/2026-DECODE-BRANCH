package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * DcMotorEx wrapper that skips redundant power writes. Every hub write costs ~2ms of loop time,
 * and subsystems re-command their motors every loop, so caching keeps the loop fast.
 */
public final class CachedMotor {
    private static final double EPSILON = 0.005;
    public final DcMotorEx motor;
    private double lastPower = Double.NaN;

    public CachedMotor(HardwareMap hw, String name, boolean reversed, boolean brake) {
        motor = hw.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we close loops ourselves; encoder still counts
    }

    public void setPower(double power) {
        double p = Math.max(-1.0, Math.min(1.0, power));
        if (Double.isNaN(lastPower) || Math.abs(p - lastPower) > EPSILON || (p == 0 && lastPower != 0)) {
            motor.setPower(p);
            lastPower = p;
        }
    }

    /** Encoder ticks; sign follows the motor direction set above. */
    public double getPosition() {
        return motor.getCurrentPosition();
    }

    /** Ticks per second; sign follows the motor direction. */
    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getLastPower() {
        return Double.isNaN(lastPower) ? 0 : lastPower;
    }
}
