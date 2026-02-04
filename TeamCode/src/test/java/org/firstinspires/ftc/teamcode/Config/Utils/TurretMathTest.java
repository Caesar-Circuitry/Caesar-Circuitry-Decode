package org.firstinspires.ftc.teamcode.Config.Utils;

import static org.junit.Assert.*;

import org.junit.Test;

/**
 * Unit tests for TurretMath utility class.
 * These tests validate angle wrapping, safety calculations, and path planning logic.
 */
public class TurretMathTest {

    private static final double DELTA = 0.001; // Tolerance for double comparisons

    // ==================== wrap180() Tests ====================

    @Test
    public void testWrap180_ZeroAngle() {
        assertEquals(0.0, TurretMath.wrap180(0.0), DELTA);
    }

    @Test
    public void testWrap180_PositiveWithinRange() {
        assertEquals(90.0, TurretMath.wrap180(90.0), DELTA);
        assertEquals(180.0, TurretMath.wrap180(180.0), DELTA);
        assertEquals(45.0, TurretMath.wrap180(45.0), DELTA);
    }

    @Test
    public void testWrap180_NegativeWithinRange() {
        assertEquals(-90.0, TurretMath.wrap180(-90.0), DELTA);
        assertEquals(-45.0, TurretMath.wrap180(-45.0), DELTA);
        // Note: wrap180 uses <= -180, so -180 wraps to 180
        assertEquals(180.0, TurretMath.wrap180(-180.0), DELTA);
    }

    @Test
    public void testWrap180_PositiveOverflow() {
        assertEquals(-90.0, TurretMath.wrap180(270.0), DELTA);
        assertEquals(0.0, TurretMath.wrap180(360.0), DELTA);
        assertEquals(45.0, TurretMath.wrap180(405.0), DELTA);
        assertEquals(0.0, TurretMath.wrap180(720.0), DELTA);
    }

    @Test
    public void testWrap180_NegativeUnderflow() {
        assertEquals(90.0, TurretMath.wrap180(-270.0), DELTA);
        assertEquals(0.0, TurretMath.wrap180(-360.0), DELTA);
        assertEquals(-45.0, TurretMath.wrap180(-405.0), DELTA);
        assertEquals(0.0, TurretMath.wrap180(-720.0), DELTA);
    }

    @Test
    public void testWrap180_EdgeCases() {
        // Test values right at the boundaries
        assertEquals(179.0, TurretMath.wrap180(179.0), DELTA);
        assertEquals(-179.0, TurretMath.wrap180(-179.0), DELTA);
        assertEquals(-1.0, TurretMath.wrap180(359.0), DELTA);
        assertEquals(1.0, TurretMath.wrap180(-359.0), DELTA);
    }

    // ==================== clamp() Tests ====================

    @Test
    public void testClamp_WithinRange() {
        assertEquals(5.0, TurretMath.clamp(5.0, 0.0, 10.0), DELTA);
        assertEquals(0.0, TurretMath.clamp(0.0, 0.0, 10.0), DELTA);
        assertEquals(10.0, TurretMath.clamp(10.0, 0.0, 10.0), DELTA);
    }

    @Test
    public void testClamp_BelowRange() {
        assertEquals(0.0, TurretMath.clamp(-5.0, 0.0, 10.0), DELTA);
        assertEquals(-10.0, TurretMath.clamp(-20.0, -10.0, 10.0), DELTA);
    }

    @Test
    public void testClamp_AboveRange() {
        assertEquals(10.0, TurretMath.clamp(15.0, 0.0, 10.0), DELTA);
        assertEquals(10.0, TurretMath.clamp(100.0, -10.0, 10.0), DELTA);
    }

    @Test
    public void testClamp_NegativeRange() {
        assertEquals(-5.0, TurretMath.clamp(-5.0, -10.0, -1.0), DELTA);
        assertEquals(-10.0, TurretMath.clamp(-15.0, -10.0, -1.0), DELTA);
        assertEquals(-1.0, TurretMath.clamp(0.0, -10.0, -1.0), DELTA);
    }

    // ==================== getSafeTurretAngle() Tests ====================

    @Test
    public void testGetSafeTurretAngle_WithinValidRange() {
        // Angles within -135 to 135 should pass through unchanged
        assertEquals(0.0, TurretMath.getSafeTurretAngle(0.0, 0.0), DELTA);
        assertEquals(90.0, TurretMath.getSafeTurretAngle(90.0, 0.0), DELTA);
        assertEquals(-90.0, TurretMath.getSafeTurretAngle(-90.0, 0.0), DELTA);
        assertEquals(135.0, TurretMath.getSafeTurretAngle(135.0, 0.0), DELTA);
        assertEquals(-135.0, TurretMath.getSafeTurretAngle(-135.0, 0.0), DELTA);
    }

    @Test
    public void testGetSafeTurretAngle_BeyondPositiveLimit() {
        // Angles beyond +135 should clamp to +135
        assertEquals(135.0, TurretMath.getSafeTurretAngle(150.0, 0.0), DELTA);
        assertEquals(135.0, TurretMath.getSafeTurretAngle(180.0, 0.0), DELTA);
        assertEquals(135.0, TurretMath.getSafeTurretAngle(170.0, 90.0), DELTA);
    }

    @Test
    public void testGetSafeTurretAngle_BeyondNegativeLimit() {
        // Angles beyond -135 should clamp to -135
        assertEquals(-135.0, TurretMath.getSafeTurretAngle(-150.0, 0.0), DELTA);
        assertEquals(-135.0, TurretMath.getSafeTurretAngle(-180.0, 0.0), DELTA);
        assertEquals(-135.0, TurretMath.getSafeTurretAngle(-170.0, -90.0), DELTA);
    }

    @Test
    public void testGetSafeTurretAngle_AtBoundaries() {
        // Exactly at boundaries
        assertEquals(135.0, TurretMath.getSafeTurretAngle(135.0, 100.0), DELTA);
        assertEquals(-135.0, TurretMath.getSafeTurretAngle(-135.0, -100.0), DELTA);
    }

    // ==================== isPathSafe() Tests ====================

    @Test
    public void testIsPathSafe_SafePathCenterToCenter() {
        double gearRatio = 2.0;
        // Current at 0°, target at 45° (both safe, with 180° offset baked in)
        // Servo angle = turret angle * gearRatio
        double currentServo = 0.0 * gearRatio + 180.0 * gearRatio; // 0° turret = 360° servo
        double targetServo = 45.0 * gearRatio + 180.0 * gearRatio; // 45° turret = 450° servo

        // Actually, the offset is already in the servo angles from AxonEncoder
        // Let's use simpler values: turret at 0° means servo at some unwrapped value
        currentServo = 0.0; // Turret at 0° (servo already has offset)
        targetServo = 90.0; // Turret at 45°

        assertTrue(TurretMath.isPathSafe(currentServo, targetServo, gearRatio));
    }

    @Test
    public void testIsPathSafe_SafePathSmallMovement() {
        double gearRatio = 2.0;
        // Small movement from 10° to 20° turret angle
        double currentServo = 20.0; // 10° turret
        double targetServo = 40.0;  // 20° turret

        assertTrue(TurretMath.isPathSafe(currentServo, targetServo, gearRatio));
    }

    @Test
    public void testIsPathSafe_UnsafeTarget_BeyondPositiveLimit() {
        double gearRatio = 2.0;
        // Target at 132° (outside 130° safe range with 5° margin)
        double currentServo = 0.0;
        double targetServo = 132.0 * gearRatio; // 264° servo = 132° turret

        assertFalse(TurretMath.isPathSafe(currentServo, targetServo, gearRatio));
    }

    @Test
    public void testIsPathSafe_UnsafeTarget_BeyondNegativeLimit() {
        double gearRatio = 2.0;
        // Target at -132° (outside -130° safe range with 5° margin)
        double currentServo = 0.0;
        double targetServo = -132.0 * gearRatio; // -264° servo = -132° turret

        assertFalse(TurretMath.isPathSafe(currentServo, targetServo, gearRatio));
    }

    @Test
    public void testIsPathSafe_UnsafeStartPosition() {
        double gearRatio = 2.0;
        // Starting outside safe range
        double currentServo = 132.0 * gearRatio; // 132° turret - outside safe zone
        double targetServo = 0.0;

        assertFalse(TurretMath.isPathSafe(currentServo, targetServo, gearRatio));
    }

    @Test
    public void testIsPathSafe_SafeFullRange() {
        double gearRatio = 2.0;
        // Path from -120° to +120° (both within safe range with margin)
        double currentServo = -120.0 * gearRatio;
        double targetServo = 120.0 * gearRatio;

        assertTrue(TurretMath.isPathSafe(currentServo, targetServo, gearRatio));
    }

    @Test
    public void testIsPathSafe_AtSafetyMarginBoundary() {
        double gearRatio = 2.0;
        // Target exactly at the safety margin boundary (135 - 5 = 130°)
        double currentServo = 0.0;
        double targetServo = 129.0 * gearRatio; // Just inside safe zone

        assertTrue(TurretMath.isPathSafe(currentServo, targetServo, gearRatio));
    }

    // ==================== getClosestServoTarget() Tests ====================

    @Test
    public void testGetClosestServoTarget_SameWrap() {
        // Current and target in same wrap, no adjustment needed
        double current = 90.0;
        double target = 45.0;
        double result = TurretMath.getClosestServoTarget(current, target);

        assertEquals(45.0, result, DELTA);
    }

    @Test
    public void testGetClosestServoTarget_WrapAroundForward() {
        // Current near 360°, target near 0° - should choose wrap forward
        double current = 350.0;
        double target = 10.0;
        double result = TurretMath.getClosestServoTarget(current, target);

        // Should choose 370° (10 + 360) since it's closer than going back to 10°
        assertEquals(370.0, result, DELTA);
    }

    @Test
    public void testGetClosestServoTarget_WrapAroundBackward() {
        // Current near 0°, target near 360° - should choose wrap backward
        double current = 10.0;
        double target = 350.0;
        double result = TurretMath.getClosestServoTarget(current, target);

        // Should choose -10° (350 - 360) since it's closer than going forward to 350°
        assertEquals(-10.0, result, DELTA);
    }

    @Test
    public void testGetClosestServoTarget_MultipleWraps() {
        // Current at 720° (2 full wraps), target should follow
        double current = 720.0;
        double target = 45.0;
        double result = TurretMath.getClosestServoTarget(current, target);

        // Should choose 720° wrap area: 45 + 720 = 765°
        assertEquals(765.0, result, DELTA);
    }

    @Test
    public void testGetClosestServoTarget_NegativeWrap() {
        // Current at -350°, target at 10°
        double current = -350.0;
        double target = 10.0;
        double result = TurretMath.getClosestServoTarget(current, target);

        // Should choose -350° (10 - 360 = -350) since current is at -350
        assertEquals(-350.0, result, DELTA);
    }

    @Test
    public void testGetClosestServoTarget_ZeroCrossing() {
        // Test crossing zero from negative to positive
        double current = -45.0;
        double target = 45.0;
        double result = TurretMath.getClosestServoTarget(current, target);

        assertEquals(45.0, result, DELTA);
    }

    // ==================== getClosestAngleInRange() Tests ====================

    @Test
    public void testGetClosestAngleInRange_WithinRange() {
        // Target within range should return unchanged
        assertEquals(0.0, TurretMath.getClosestAngleInRange(0.0, -135.0, 135.0), DELTA);
        assertEquals(90.0, TurretMath.getClosestAngleInRange(90.0, -135.0, 135.0), DELTA);
        assertEquals(-90.0, TurretMath.getClosestAngleInRange(-90.0, -135.0, 135.0), DELTA);
    }

    @Test
    public void testGetClosestAngleInRange_BeyondMax() {
        // Target beyond max, should snap to closest boundary
        double result = TurretMath.getClosestAngleInRange(150.0, -135.0, 135.0);
        // 150° is 15° past max, should snap to max (135°)
        assertEquals(135.0, result, DELTA);
    }

    @Test
    public void testGetClosestAngleInRange_BeyondMin() {
        // Target beyond min, should snap to closest boundary
        double result = TurretMath.getClosestAngleInRange(-150.0, -135.0, 135.0);
        // -150° is 15° past min, should snap to min (-135°)
        assertEquals(-135.0, result, DELTA);
    }

    // ==================== Integration/Edge Case Tests ====================

    @Test
    public void testWrap180_ConsistencyWithAxonEncoder() {
        // Verify wrap180 produces consistent results for typical encoder values
        // AxonEncoder uses the same while-loop implementation

        // Simulate encoder reading at various positions
        double[] testAngles = {0, 90, 180, 270, 360, -90, -180, -270, -360, 450, -450};
        double[] expected = {0, 90, 180, -90, 0, -90, 180, 90, 0, 90, -90};

        for (int i = 0; i < testAngles.length; i++) {
            assertEquals("Failed for angle " + testAngles[i],
                    expected[i], TurretMath.wrap180(testAngles[i]), DELTA);
        }
    }

    @Test
    public void testSafetyChain_DesiredToSafeToPath() {
        // Test the full chain: desired angle -> safe angle -> path check
        double gearRatio = 2.0;
        double heading = 30.0; // Robot heading
        double fieldTarget = 120.0; // Field-relative target

        // Calculate robot-relative desired angle
        double desiredTurret = TurretMath.wrap180(fieldTarget - heading); // 90°

        // Get safe angle
        double safeTurret = TurretMath.getSafeTurretAngle(desiredTurret, 0.0);

        // Convert to servo angle
        double targetServo = safeTurret * gearRatio;

        // Check path safety
        double currentServo = 0.0;
        double closestTarget = TurretMath.getClosestServoTarget(currentServo, targetServo);

        assertTrue("Path should be safe for normal operation",
                TurretMath.isPathSafe(currentServo, closestTarget, gearRatio));
    }

    @Test
    public void testSafetyChain_UnsafeDesiredAngle() {
        // Test that unsafe desired angles get properly handled
        double gearRatio = 2.0;
        double heading = 0.0;
        double fieldTarget = 170.0; // Would result in 170° robot-relative (unsafe)

        // Calculate robot-relative desired angle
        double desiredTurret = TurretMath.wrap180(fieldTarget - heading); // 170°

        // Get safe angle - should clamp to 135°
        double safeTurret = TurretMath.getSafeTurretAngle(desiredTurret, 0.0);
        assertEquals(135.0, safeTurret, DELTA);

        // The clamped angle should result in a safe path
        double targetServo = safeTurret * gearRatio;
        double currentServo = 0.0;
        double closestTarget = TurretMath.getClosestServoTarget(currentServo, targetServo);

        assertTrue("Clamped angle should produce safe path",
                TurretMath.isPathSafe(currentServo, closestTarget, gearRatio));
    }
}
