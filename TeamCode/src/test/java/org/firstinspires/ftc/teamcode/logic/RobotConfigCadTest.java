package org.firstinspires.ftc.teamcode.logic;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.junit.Test;

/**
 * Pins RobotConfig to the numbers read from the team's Fusion CAD ("Robot" assembly, project
 * BFC 2025-2026). If the robot is rebuilt, update the CAD numbers here and in RobotConfig together.
 */
public class RobotConfigCadTest {
    @Test
    public void driveEncoderScaleMatchesRevHdHexWithUltraPlanetary4And5() {
        // REV: 4:1 cartridge = 76:21, 5:1 cartridge = 68:13; HD Hex encoder 28 counts per motor rev.
        assertEquals(18.930, RobotConfig.DRIVE_GEAR_RATIO, 0.001);
        assertEquals(530.05, RobotConfig.DRIVE_TICKS_PER_WHEEL_REV, 0.05);
        // 104mm mecanum, 1:1 belt.
        double inchesPerWheelRev = 104 / 25.4 * Math.PI;
        assertEquals(inchesPerWheelRev, RobotConfig.DRIVE_INCHES_PER_TICK * RobotConfig.DRIVE_TICKS_PER_WHEEL_REV, 1e-9);
        assertEquals(0.024268, RobotConfig.DRIVE_INCHES_PER_TICK, 0.000001);
        // 6000rpm / 18.93 = 317rpm at the wheel -> ~68 in/s free.
        assertEquals(68.0, RobotConfig.DRIVE_FREE_SPEED_IN_PER_S, 0.5);
    }

    @Test
    public void footprintFromCad() {
        assertEquals(18.0, RobotConfig.ROBOT_WIDTH, 0.05);
        assertEquals(18.94, RobotConfig.ROBOT_LENGTH, 0.05);
    }

    @Test
    public void sensorsSitInsideTheFootprint() {
        double hl = RobotConfig.ROBOT_LENGTH / 2, hw = RobotConfig.ROBOT_WIDTH / 2;
        assertTrue(Math.abs(RobotConfig.LIMELIGHT_FORWARD_IN) < hl);
        assertTrue(Math.abs(RobotConfig.LIMELIGHT_LEFT_IN) < hw);
        assertTrue(RobotConfig.LIMELIGHT_HEIGHT_IN > 0 && RobotConfig.LIMELIGHT_HEIGHT_IN < FieldGeometry.GOAL_TAG_HEIGHT);
        assertTrue(Math.abs(RobotConfig.PINPOINT_X_POD_OFFSET_MM) / 25.4 < hw);
        assertTrue(Math.abs(RobotConfig.PINPOINT_Y_POD_OFFSET_MM) / 25.4 < hl);
    }

    @Test
    public void aimGainsCarryTheCameraMount() {
        AimController.Gains g = RobotConfig.aimGains();
        assertEquals(RobotConfig.LIMELIGHT_FORWARD_IN, g.cameraForward, 0);
        assertEquals(RobotConfig.LIMELIGHT_LEFT_IN, g.cameraLeft, 0);
        assertEquals(g.cameraLeft, RobotConfig.autoSettings().aim.cameraLeft, 0);
    }
}
