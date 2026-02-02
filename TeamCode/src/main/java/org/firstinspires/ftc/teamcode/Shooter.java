package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() {}

    private final MotorEx shooterMotor = new MotorEx("shooterMotor");
    private final ServoEx left  = new ServoEx("shooterAngle2");
    private final ServoEx right = new ServoEx("shooterAngle1");
    private final ServoEx picker = new ServoEx("pickerServo");

    // Hood
    private double hoodPos = 0.5;
    private static final double STEP = 0.1;
    private static final double MIN_POS = 0.10;
    private static final double MAX_POS = 0.90;

    // Shooter motor
    private static final double SHOOT_POWER = 0.8;

    // Picker
    private static final double PICKER_DOWN = 0.20;
    private static final double PICKER_UP   = 0.45;
    private double pickerTarget = PICKER_DOWN;

    // Shooter
    public final Command shooterOn = new LambdaCommand()
            .setStart(() -> {
                shooterMotor.setPower(SHOOT_POWER);
                ActiveOpMode.telemetry().addData("Shooter", "Motor ON %.2f", SHOOT_POWER);
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Shooter Motor On");

    public final Command shooterOff = new LambdaCommand()
            .setStart(() -> {
                shooterMotor.setPower(0.0);
                ActiveOpMode.telemetry().addData("Shooter", "Motor OFF");
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Shooter Motor Off");

    // Picker
    public final Command pickerUp = new LambdaCommand()
            .setStart(() -> {
                pickerTarget = PICKER_UP;
                ActiveOpMode.telemetry().addData("Shooter", "Picker UP %.2f", PICKER_UP);
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Picker Up");

    public final Command pickerDown = new LambdaCommand()
            .setStart(() -> {
                pickerTarget = PICKER_DOWN;
                ActiveOpMode.telemetry().addData("Shooter", "Picker DOWN %.2f", PICKER_DOWN);
                ActiveOpMode.telemetry().update();
            })
            .setIsDone(() -> true)
            .requires(this)
            .named("Picker Down");

    // Hood angle
    public final Command angleUp = new LambdaCommand()
            .setStart(() -> setHood(hoodPos + STEP))
            .setIsDone(() -> true)
            .requires(this)
            .named("Hood Angle Up");

    public final Command angleDown = new LambdaCommand()
            .setStart(() -> setHood(hoodPos - STEP))
            .setIsDone(() -> true)
            .requires(this)
            .named("Hood Angle Down");

    private void setHood(double pos) {
        hoodPos = clamp(pos, MIN_POS, MAX_POS);
        ActiveOpMode.telemetry().addData("Shooter", "Hood %.3f", hoodPos);
        ActiveOpMode.telemetry().update();
    }

    @Override
    public void periodic() {
        // enforce hood every loop (mirrored linkage)
        left.setPosition(hoodPos);
        right.setPosition(1.0 - hoodPos);

        // enforce picker every loop
        picker.setPosition(pickerTarget);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
