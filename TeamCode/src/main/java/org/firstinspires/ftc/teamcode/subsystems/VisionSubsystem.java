package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.logic.Alliance;
import org.firstinspires.ftc.teamcode.logic.Motif;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Limelight 3A AprilTag vision: goal aim (tx), goal range, and OBELISK motif.
 *
 * <p>Optional hardware: if no device called "limelight" is configured the subsystem reports
 * "no target" forever and everything else keeps working on odometry.
 */
public final class VisionSubsystem implements Subsystem {
    public static final VisionSubsystem INSTANCE = new VisionSubsystem();

    /** A goal tag sighting. */
    public static final class GoalSighting {
        public final int tagId;
        public final double txDegrees;
        /** Camera-to-tag straight-line range in inches (independent of Limelight mount settings). */
        public final double rangeInches;
        public final double area;

        GoalSighting(int tagId, double txDegrees, double rangeInches, double area) {
            this.tagId = tagId;
            this.txDegrees = txDegrees;
            this.rangeInches = rangeInches;
            this.area = area;
        }
    }

    private static final long STALE_MS = 250;

    private Limelight3A limelight;
    private LLResult result;
    private double yawDegrees;

    private VisionSubsystem() {}

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().tryGet(Limelight3A.class, RobotConfig.LIMELIGHT);
        result = null;
        if (limelight != null) {
            limelight.pipelineSwitch(RobotConfig.LIMELIGHT_APRILTAG_PIPELINE);
            limelight.setPollRateHz(100);
            limelight.start();
        }
    }

    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    /** Robot yaw in degrees for MegaTag2 localisation; set by the OpMode every loop. */
    public void setRobotYawDegrees(double yaw) {
        yawDegrees = yaw;
    }

    @Override
    public void periodic() {
        if (limelight == null) return;
        limelight.updateRobotOrientation(yawDegrees);
        LLResult r = limelight.getLatestResult();
        result = (r != null && r.isValid() && r.getStaleness() < STALE_MS) ? r : null;
    }

    /**
     * @param alliance which goal to look for, or null to accept either goal (biggest wins)
     * @return the sighting, or null when the tag is not visible
     */
    public GoalSighting getGoal(Alliance alliance) {
        if (result == null) return null;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return null;
        GoalSighting best = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            boolean isGoal = id == Alliance.RED.goalTagId || id == Alliance.BLUE.goalTagId;
            if (!isGoal || (alliance != null && id != alliance.goalTagId)) continue;
            double range = Double.NaN;
            Pose3D camToTag = t.getTargetPoseCameraSpace();
            if (camToTag != null) {
                Position p = camToTag.getPosition().toUnit(DistanceUnit.INCH);
                range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            }
            GoalSighting g = new GoalSighting(id, t.getTargetXDegrees(), range, t.getTargetArea());
            if (best == null || g.area > best.area) best = g;
        }
        return best;
    }

    /** @return the obelisk motif if an obelisk tag is visible, else null */
    public Motif getMotif() {
        if (result == null) return null;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return null;
        for (LLResultTypes.FiducialResult t : tags) {
            Motif m = Motif.fromTagId(t.getFiducialId());
            if (m != null) return m;
        }
        return null;
    }

    /** MegaTag2 field position in inches {x, y}, or null. Frame must match the SDK field frame. */
    public double[] getFieldPosition() {
        if (result == null) return null;
        Pose3D bot = result.getBotpose_MT2();
        if (bot == null || result.getBotposeTagCount() == 0) return null;
        Position p = bot.getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.y == 0) return null;
        return new double[] {p.x, p.y};
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
