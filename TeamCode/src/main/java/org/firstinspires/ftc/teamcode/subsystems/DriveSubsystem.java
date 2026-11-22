package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.MathUtil;
import org.firstinspires.ftc.teamcode.logic.MecanumKinematics;
import org.firstinspires.ftc.teamcode.logic.MecanumOdometry;
import org.firstinspires.ftc.teamcode.logic.Pose2d;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Mecanum drivetrain and localisation.
 *
 * <p>Pose source, best first:
 * <ol>
 *   <li>goBILDA Pinpoint with the two dead-wheel pods (in the CAD), if configured as "pinpoint" and healthy;</li>
 *   <li>drive-motor encoders + Control Hub IMU ({@link MecanumOdometry}), always running in the background.</li>
 * </ol>
 * If the Pinpoint reports a fault mid-match the drive switches to the encoder pose, seeded with the last
 * good Pinpoint pose.
 */
public final class DriveSubsystem implements Subsystem {
    public static final DriveSubsystem INSTANCE = new DriveSubsystem();

    /** Pose at the end of the last OpMode, so TeleOp can continue from where AUTO finished. */
    private static Pose2d lastPose = null;

    private CachedMotor fl, fr, bl, br;
    private IMU imu;
    private MecanumOdometry odometry;
    private GoBildaPinpointDriver pinpoint;
    /** Pose to write into the Pinpoint once it has finished calibrating. */
    private Pose2d pinpointPending;
    private boolean usingPinpoint;
    private Pose2d pose = new Pose2d(0, 0, 0);
    private double forward, left, turn;

    private DriveSubsystem() {}

    @Override
    public void initialize() {
        HardwareMap hw = ActiveOpMode.hardwareMap();
        fl = new CachedMotor(hw, RobotConfig.LEFT_FRONT, RobotConfig.LEFT_REVERSED, true);
        bl = new CachedMotor(hw, RobotConfig.LEFT_BACK, RobotConfig.LEFT_REVERSED, true);
        fr = new CachedMotor(hw, RobotConfig.RIGHT_FRONT, RobotConfig.RIGHT_REVERSED, true);
        br = new CachedMotor(hw, RobotConfig.RIGHT_BACK, RobotConfig.RIGHT_REVERSED, true);

        imu = hw.get(IMU.class, RobotConfig.IMU);
        // Same hub orientation the competition code used.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        odometry = new MecanumOdometry(RobotConfig.DRIVE_INCHES_PER_TICK, RobotConfig.LATERAL_MULTIPLIER);
        Pose2d start = lastPose != null ? lastPose : new Pose2d(0, 0, 0);
        odometry.setPose(start, 0.0);
        pose = start;
        forward = left = turn = 0;

        pinpoint = hw.tryGet(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
        usingPinpoint = false;
        pinpointPending = null;
        if (pinpoint != null) {
            pinpoint.setOffsets(RobotConfig.PINPOINT_X_POD_OFFSET_MM, RobotConfig.PINPOINT_Y_POD_OFFSET_MM,
                    DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    RobotConfig.PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    RobotConfig.PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            // The robot is stationary during INIT: zero the pods and calibrate the gyro (~0.25s, async).
            pinpoint.resetPosAndIMU();
            pinpointPending = start;
        }
    }

    /** Robot-centric command: forward +, strafe left +, turn counter-clockwise +. */
    public void drive(double forward, double left, double turnCcw) {
        this.forward = forward;
        this.left = left;
        this.turn = turnCcw;
    }

    /** Field-centric command: translation in the field frame (driver's view), turn CCW +. */
    public void driveFieldCentric(double fieldX, double fieldY, double turnCcw) {
        double[] r = MecanumKinematics.fieldToRobot(fieldX, fieldY, pose.heading);
        drive(r[0], r[1], turnCcw);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public void periodic() {
        Pose2d encoderPose = odometry.update(
                new double[] {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()}, imuHeading());

        boolean wasUsingPinpoint = usingPinpoint;
        usingPinpoint = false;
        if (pinpoint != null) {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                if (pinpointPending != null) {
                    writePinpoint(pinpointPending);
                    pinpointPending = null;
                } else {
                    Pose2D p = pinpoint.getPosition();
                    double x = p.getX(DistanceUnit.INCH), y = p.getY(DistanceUnit.INCH);
                    double h = p.getHeading(AngleUnit.RADIANS);
                    if (MathUtil.isFinite(x) && MathUtil.isFinite(y) && MathUtil.isFinite(h)) {
                        pose = new Pose2d(x, y, MathUtil.wrapAngle(h));
                        usingPinpoint = true;
                    }
                }
            }
        }
        if (!usingPinpoint) {
            if (wasUsingPinpoint) {
                // Pinpoint just dropped out: carry on from its last good pose on the motor encoders.
                odometry.setPose(pose, imuHeading());
            } else {
                // No Pinpoint, still calibrating (encoders were seeded with the same pose), or faulted.
                pose = encoderPose;
            }
        }
        lastPose = pose;

        MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(forward, left, turn);
        fl.setPower(w.fl);
        fr.setPower(w.fr);
        bl.setPower(w.bl);
        br.setPower(w.br);
    }

    private void writePinpoint(Pose2d p) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.RADIANS, p.heading));
        pose = p;
    }

    private double imuHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public Pose2d getPose() {
        return pose;
    }

    /** Human-readable pose source for telemetry. */
    public String getPoseSource() {
        if (pinpoint == null) return "encoders + IMU (no \"" + RobotConfig.PINPOINT + "\" configured)";
        if (usingPinpoint) return "Pinpoint";
        GoBildaPinpointDriver.DeviceStatus st = pinpoint.getDeviceStatus();
        if (st == GoBildaPinpointDriver.DeviceStatus.CALIBRATING || st == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            return "Pinpoint calibrating, encoders meanwhile";
        }
        return "Pinpoint " + st + " -> encoders + IMU";
    }

    /** Sets the full field pose (start of AUTO). */
    public void setPose(Pose2d p) {
        odometry.setPose(p, imuHeading());
        if (pinpoint != null) {
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                writePinpoint(p);
                pinpointPending = null;
            } else {
                pinpointPending = p;
            }
        }
        pose = p;
        lastPose = p;
    }

    /** Re-seeds x/y (e.g. from vision) without changing heading. */
    public void setPosition(double x, double y) {
        setPose(new Pose2d(x, y, pose.heading));
    }

    /** Driver "reset heading": the direction the robot faces now becomes field heading {@code heading}. */
    public void resetHeading(double heading) {
        setPose(new Pose2d(pose.x, pose.y, heading));
    }

    public double[] getWheelPowers() {
        return new double[] {fl.getLastPower(), fr.getLastPower(), bl.getLastPower(), br.getLastPower()};
    }

    public static void clearStoredPose() {
        lastPose = null;
    }
}
