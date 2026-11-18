package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.MecanumKinematics;
import org.firstinspires.ftc.teamcode.logic.MecanumOdometry;
import org.firstinspires.ftc.teamcode.logic.Pose2d;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/** Mecanum drivetrain + IMU + encoder odometry. */
public final class DriveSubsystem implements Subsystem {
    public static final DriveSubsystem INSTANCE = new DriveSubsystem();

    /** Pose at the end of the last OpMode, so TeleOp can continue from where AUTO finished. */
    private static Pose2d lastPose = null;

    private CachedMotor fl, fr, bl, br;
    private IMU imu;
    private MecanumOdometry odometry;
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
        pose = odometry.update(new double[] {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()},
                imuHeading());
        lastPose = pose;

        MecanumKinematics.WheelPowers w = MecanumKinematics.robotCentric(forward, left, turn);
        fl.setPower(w.fl);
        fr.setPower(w.fr);
        bl.setPower(w.bl);
        br.setPower(w.br);
    }

    private double imuHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public Pose2d getPose() {
        return pose;
    }

    /** Sets the full field pose (start of AUTO). */
    public void setPose(Pose2d p) {
        odometry.setPose(p, imuHeading());
        pose = p;
        lastPose = p;
    }

    /** Re-seeds x/y from vision without touching heading. */
    public void setPosition(double x, double y) {
        odometry.setPosition(x, y);
        pose = new Pose2d(x, y, pose.heading);
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
