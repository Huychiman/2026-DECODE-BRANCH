package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Commands.FieldCentricMecanumDrive;

//import org.firstinspires.ftc.teamcode.Mechanisms.IMUtest;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Picker;
import org.firstinspires.ftc.teamcode.Mechanisms.Rearranger;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name="TeleOpMain")
    public class TeleOpMain extends NextFTCOpMode{

    private final MotorEx frontLeftMotor  = new MotorEx("leftFront").reversed();
    private final MotorEx frontRightMotor = new MotorEx("rightFront");
    private final MotorEx backLeftMotor   = new MotorEx("leftBack").reversed();
    private final MotorEx backRightMotor  = new MotorEx("rightBack");

    private IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();
    //private double driverScale = 0.9;


    public TeleOpMain() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Rearranger.INSTANCE, Shooter.INSTANCE, Picker.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }

    @Override
    public void onInit() {
       // imu = new IMUEx("imu", Direction.LEFT, Direction.UP).zeroed();
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().map(v -> v * 0.9), // front and back movement
                Gamepads.gamepad1().leftStickX().negate().map(v -> v * 0.9), // left and right movement
                Gamepads.gamepad1().rightStickX().negate().map(v -> v * 0.9)// spin
                //new FieldCentric(imu)
        );
        driverControlled.schedule();

        /*Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(new LambdaCommand()
                        .setStart(() -> driverScale = 0.6)
                        .setIsDone(() -> true)
                );

        Gamepads.gamepad1().leftBumper()
                .whenBecomesFalse(new LambdaCommand()
                        .setStart(() -> driverScale = 0.9) // normal speed
                        .setIsDone(() -> true)
                );*/

        // Intake
        Gamepads.gamepad1().triangle().whenBecomesTrue(Intake.INSTANCE.toggle);

        // Rearrangement
        Gamepads.gamepad1().square().whenBecomesTrue(Rearranger.INSTANCE.left);
        Gamepads.gamepad1().circle().whenBecomesTrue(Rearranger.INSTANCE.right);

        // shoot
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Shooter.INSTANCE.shooterOn);
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(Shooter.INSTANCE.shooterOff);

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Picker.INSTANCE.pickerDown);
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(Picker.INSTANCE.pickerUp);

        // Shooter hood
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(Shooter.INSTANCE.angleUp);
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(Shooter.INSTANCE.angleDown);


    }
}
