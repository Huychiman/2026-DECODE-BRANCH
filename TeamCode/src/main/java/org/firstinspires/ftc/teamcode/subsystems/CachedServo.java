package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Servo wrapper that only writes when the commanded position actually changes. */
public final class CachedServo {
    public final Servo servo;
    private double last = Double.NaN;

    public CachedServo(HardwareMap hw, String name) {
        servo = hw.get(Servo.class, name);
    }

    public void setPosition(double position) {
        double p = Math.max(0.0, Math.min(1.0, position));
        if (Double.isNaN(last) || Math.abs(p - last) > 0.001) {
            servo.setPosition(p);
            last = p;
        }
    }

    public double getCommanded() {
        return last;
    }
}
