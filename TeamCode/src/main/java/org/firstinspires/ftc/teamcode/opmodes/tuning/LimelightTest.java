package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.Motif;

/** Shows every AprilTag the Limelight sees, the motif, and MegaTag2 pose. (Was AprilTagLimelightTest.) */
@TeleOp(name = "Tune: Limelight", group = "Tune")
public class LimelightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
        limelight.pipelineSwitch(RobotConfig.LIMELIGHT_APRILTAG_PIPELINE);
        imu = hardwareMap.get(IMU.class, RobotConfig.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult r = limelight.getLatestResult();
        telemetry.addData("Connected", limelight.isConnected());
        if (r == null || !r.isValid()) {
            telemetry.addLine("No valid result");
            return;
        }
        telemetry.addData("Pipeline", r.getPipelineIndex());
        telemetry.addData("Staleness", "%d ms", r.getStaleness());
        for (LLResultTypes.FiducialResult t : r.getFiducialResults()) {
            Motif m = Motif.fromTagId(t.getFiducialId());
            String what = m != null ? "OBELISK " + m : t.getFiducialId() == 24 ? "RED GOAL" : t.getFiducialId() == 20 ? "BLUE GOAL" : "?";
            Position p = t.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            telemetry.addData("Tag " + t.getFiducialId(), "%s tx %.1f ty %.1f area %.2f range %.1fin", what,
                    t.getTargetXDegrees(), t.getTargetYDegrees(), t.getTargetArea(),
                    Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
        }
        Pose3D bot = r.getBotpose_MT2();
        if (bot != null) {
            Position p = bot.getPosition().toUnit(DistanceUnit.INCH);
            telemetry.addData("MT2 pose", "(%.1f, %.1f) in  yaw %.1f°", p.x, p.y, bot.getOrientation().getYaw(AngleUnit.DEGREES));
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
