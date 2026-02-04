package org.firstinspires.ftc.teamcode.Config.Utils;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests for AnglePIDF controller.
 * Tests PIDF calculations, angle wrapping, and danger zone limiting.
 */
public class AnglePIDFTest {

    private static final double DELTA = 0.001; // Tolerance for double comparisons
    private AnglePIDF controller;

    @Before
    public void setUp() {
        // Create a controller with known gains for testing
        controller = new AnglePIDF(0.008, 0.0, 0.0, 0.07, -0.06);
    }

    // ==================== Basic PIDF Tests ====================

    @Test
    public void testSimpleCalculation() {
        // Create a fresh controller to avoid any setup issues
        AnglePIDF testController = new AnglePIDF(0.0, 0.1, 0.0, 0.0, 0.0);
        testController.setIntegralMax(100.0); // Set high enough to avoid clamping
        testController.setSetPoint(10.0);

        // First call: setpoint=10, measurement=0, error should be 10
        double output = testController.calculate(0.0);
        double integral = testController.getIntegral();
        double lastError = testController.getLastError();

        System.out.println("Simple test: setpoint=10, measurement=0");
        System.out.println("  output=" + output + ", integral=" + integral + ", lastError=" + lastError);

        // Expected: error = 10, integral = 10, output = 0.1 * 10 = 1.0
        assertEquals("Error should be 10", 10.0, lastError, DELTA);
        assertEquals("Integral should be 10", 10.0, integral, DELTA);
        assertEquals("Output should be 1.0", 1.0, output, DELTA);
    }

    @Test
    public void testInitialization() {
        assertNotNull(controller);
        assertEquals(0.0, controller.getSetPoint(), DELTA);
        assertEquals(0.0, controller.getLastError(), DELTA);
        assertEquals(0.0, controller.getIntegral(), DELTA);
    }

    @Test
    public void testSetSetPoint() {
        controller.setSetPoint(45.0);
        assertEquals(45.0, controller.getSetPoint(), DELTA);

        controller.setSetPoint(-90.0);
        assertEquals(-90.0, controller.getSetPoint(), DELTA);
    }

    @Test
    public void testProportionalControl_PositiveError() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        controller.setSetPoint(100.0);

        double output = controller.calculate(50.0);

        // Error = 100 - 50 = 50, output = 0.1 * 50 = 5.0
        assertEquals(5.0, output, DELTA);
    }

    @Test
    public void testProportionalControl_NegativeError() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        controller.setSetPoint(50.0);

        double output = controller.calculate(100.0);

        // Error = 50 - 100 = -50, output = 0.1 * (-50) = -5.0
        assertEquals(-5.0, output, DELTA);
    }

    @Test
    public void testProportionalControl_ZeroError() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        controller.setSetPoint(90.0);

        double output = controller.calculate(90.0);

        assertEquals(0.0, output, DELTA);
    }

    // ==================== Angle Wrapping Tests ====================

    @Test
    public void testAngleWrapping_PositiveWrap() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        // Current at 170°, target at -170°
        // Shortest path is +20° (through 180), not -340°
        controller.setSetPoint(-170.0);

        double output = controller.calculate(170.0);

        // Error should be wrapped to 20° (going positive direction)
        // output = 0.1 * 20 = 2.0
        assertEquals(2.0, output, DELTA);
    }

    @Test
    public void testAngleWrapping_NegativeWrap() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        // Current at -170°, target at 170°
        // Shortest path is -20° (through -180), not +340°
        controller.setSetPoint(170.0);

        double output = controller.calculate(-170.0);

        // Error should be wrapped to -20° (going negative direction)
        // output = 0.1 * (-20) = -2.0
        assertEquals(-2.0, output, DELTA);
    }

    @Test
    public void testAngleWrapping_SmallPositiveMove() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        // Current at 175°, target at -175°
        // Shortest path is +10°
        controller.setSetPoint(-175.0);

        double output = controller.calculate(175.0);

        // Error = 10°, output = 0.1 * 10 = 1.0
        assertEquals(1.0, output, DELTA);
    }

    @Test
    public void testAngleWrapping_LargeAngleValues() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        // Test with unwrapped angles
        controller.setSetPoint(450.0); // Same as 90°

        double output = controller.calculate(360.0); // Same as 0°

        // Error should be 90° (wrapped), output = 0.1 * 90 = 9.0
        assertEquals(9.0, output, DELTA);
    }

    // ==================== Integral Control Tests ====================

    @Test
    public void testIntegralControl_Accumulates() {
        controller = new AnglePIDF(0.0, 0.1, 0.0, 0.0, 0.0);
        controller.setIntegralMax(100.0); // Set high enough to avoid clamping
        controller.setSetPoint(10.0);

        // First calculation - error = wrap180(10.0 - 0.0) = 10.0
        double output1 = controller.calculate(0.0);
        double actualIntegral1 = controller.getIntegral();
        double actualError1 = controller.getLastError();

        System.out.println("First call: output=" + output1 + ", integral=" + actualIntegral1 + ", error=" + actualError1);
        assertEquals("First integral should be 10.0, but got " + actualIntegral1, 10.0, actualIntegral1, DELTA);
        assertEquals("First output should be 1.0, but got " + output1, 1.0, output1, DELTA); // 0.1 * 10 = 1.0

        // Second calculation - same error = 10.0, integral accumulates to 20.0
        double output2 = controller.calculate(0.0);
        double actualIntegral2 = controller.getIntegral();
        double actualError2 = controller.getLastError();

        System.out.println("Second call: output=" + output2 + ", integral=" + actualIntegral2 + ", error=" + actualError2);
        assertEquals("Second integral should be 20.0, but got " + actualIntegral2, 20.0, actualIntegral2, DELTA);
        assertEquals("Second output should be 2.0, but got " + output2, 2.0, output2, DELTA); // 0.1 * 20 = 2.0
    }

    @Test
    public void testIntegralControl_AntiWindup() {
        controller = new AnglePIDF(0.0, 0.1, 0.0, 0.0, 0.0);
        controller.setIntegralMax(50.0);
        controller.setSetPoint(100.0);

        // Run many iterations to try to exceed integral max
        for (int i = 0; i < 100; i++) {
            controller.calculate(0.0);
        }

        // Integral should be clamped
        assertTrue("Integral should be clamped to max",
                Math.abs(controller.getIntegral()) <= 50.0);
    }

    @Test
    public void testIntegralControl_NegativeAccumulation() {
        controller = new AnglePIDF(0.0, 0.1, 0.0, 0.0, 0.0);
        controller.setIntegralMax(100.0); // Set high enough to avoid clamping
        controller.setSetPoint(-10.0);

        // Error = wrap180(-10.0 - 0.0) = -10.0
        double output = controller.calculate(0.0);
        double actualIntegral = controller.getIntegral();
        double actualError = controller.getLastError();

        System.out.println("Negative test: output=" + output + ", integral=" + actualIntegral + ", error=" + actualError);
        assertEquals("Integral should be -10.0, but got " + actualIntegral, -10.0, actualIntegral, DELTA);
        assertEquals("Output should be -1.0, but got " + output, -1.0, output, DELTA); // 0.1 * -10 = -1.0
    }

    // ==================== Derivative Control Tests ====================

    @Test
    public void testDerivativeControl_FirstRunZero() {
        controller = new AnglePIDF(0.0, 0.0, 1.0, 0.0, 0.0);
        controller.setSetPoint(10.0);

        // First run should have no derivative contribution
        double output = controller.calculate(0.0);

        // Derivative is 0 on first run
        assertEquals(0.0, output, DELTA);
    }

    @Test
    public void testDerivativeControl_ErrorChanging() {
        controller = new AnglePIDF(0.0, 0.0, 0.1, 0.0, 0.0);
        controller.setSetPoint(50.0);

        // First calculation (error = 50)
        controller.calculate(0.0);

        // Second calculation (error = 40, change = -10)
        double output = controller.calculate(10.0);

        // Derivative = 0.1 * (-10) = -1.0
        assertEquals(-1.0, output, DELTA);
    }

    // ==================== Feedforward Tests ====================

    @Test
    public void testFeedforward_PositiveDirection() {
        controller = new AnglePIDF(0.0, 0.0, 0.0, 0.1, 0.0);
        controller.setSetPoint(10.0);

        double output = controller.calculate(0.0);

        // Error is positive (10), so kF_left should be added
        assertEquals(0.1, output, DELTA);
    }

    @Test
    public void testFeedforward_NegativeDirection() {
        controller = new AnglePIDF(0.0, 0.0, 0.0, 0.0, -0.1);
        controller.setSetPoint(-10.0);

        double output = controller.calculate(0.0);

        // Error is negative (-10), so kF_right should be added
        assertEquals(-0.1, output, DELTA);
    }

    @Test
    public void testFeedforward_NoMovement() {
        controller = new AnglePIDF(0.0, 0.0, 0.0, 0.1, -0.1);
        controller.setSetPoint(0.0);

        double output = controller.calculate(0.0);

        // Error is 0, no feedforward should be added
        assertEquals(0.0, output, DELTA);
    }

    @Test
    public void testFeedforward_CombinedWithP() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.05, -0.05);
        controller.setSetPoint(10.0);

        double output = controller.calculate(0.0);

        // P: 0.1 * 10 = 1.0
        // F: 0.05 (positive error)
        // Total: 1.05
        assertEquals(1.05, output, DELTA);
    }

    // ==================== Reset Tests ====================

    @Test
    public void testReset_ClearsState() {
        controller = new AnglePIDF(0.1, 0.1, 0.1, 0.0, 0.0);
        controller.setSetPoint(10.0);

        // Run some calculations
        controller.calculate(0.0);
        controller.calculate(5.0);

        // Reset
        controller.reset();

        assertEquals(0.0, controller.getLastError(), DELTA);
        assertEquals(0.0, controller.getIntegral(), DELTA);
    }

    @Test
    public void testReset_DerivativeIsZeroAfterReset() {
        controller = new AnglePIDF(0.0, 0.0, 1.0, 0.0, 0.0);
        controller.setSetPoint(10.0);

        // Run calculations to build up derivative state
        controller.calculate(0.0);
        controller.calculate(5.0);

        // Reset
        controller.reset();

        // After reset, first run should have no derivative
        double output = controller.calculate(0.0);
        assertEquals(0.0, output, DELTA);
    }

    // ==================== Danger Zone Tests ====================

    @Test
    public void testDangerZone_Disabled() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        // Danger zone limiting is disabled by default

        controller.setSetPoint(150.0); // In typical danger zone
        double output = controller.calculate(0.0);

        // Should calculate normally
        assertEquals(15.0, output, DELTA); // 0.1 * 150 = 15
    }

    @Test
    public void testDangerZone_Enabled_SafePath() {
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        controller.enableDangerZoneLimiting();
        controller.setDangerZone(135.0, 180.0); // Danger zone is 135° to 180°

        controller.setSetPoint(90.0); // Safe target
        double output = controller.calculate(0.0);

        // Path from 0 to 90 doesn't enter danger zone
        assertEquals(9.0, output, DELTA); // 0.1 * 90 = 9
    }

    @Test
    public void testSetCoefficients() {
        controller.setCoefficients(0.2, 0.05, 0.01, 0.1, -0.1);
        controller.setIntegralMax(100.0); // Set high enough to avoid clamping
        controller.setSetPoint(10.0);

        double output = controller.calculate(0.0);
        double actualIntegral = controller.getIntegral();
        double actualError = controller.getLastError();

        System.out.println("SetCoefficients test: output=" + output + ", integral=" + actualIntegral + ", error=" + actualError);

        // Error = wrap180(10.0 - 0.0) = 10.0
        // P: 0.2 * 10 = 2.0
        // I: 0.05 * 10 = 0.5 (first iteration accumulates error)
        // D: 0.01 * 0 = 0 (first iteration has no derivative)
        // F: 0.1 (positive error)
        // Total: 2.0 + 0.5 + 0 + 0.1 = 2.6
        assertEquals("Output should be 2.6, but got " + output, 2.6, output, DELTA);
        assertEquals("Integral should be 10.0, but got " + actualIntegral, 10.0, actualIntegral, DELTA);
        assertEquals("Last error should be 10.0, but got " + actualError, 10.0, actualError, DELTA);
    }

    // ==================== Integration Tests ====================


    @Test
    public void testTurretScenario_FieldRelativeTracking() {
        // Simulate turret tracking a field-relative target as robot rotates
        controller = new AnglePIDF(0.01, 0.0, 0.0, 0.0, 0.0);

        double fieldTarget = 45.0; // Point at 45° field-relative
        double robotHeading = 0.0; // Robot facing 0°
        double turretAngle = 0.0; // Turret facing forward

        // Calculate desired turret angle (field target relative to robot heading)
        double desiredTurret = fieldTarget - robotHeading; // 45°
        controller.setSetPoint(desiredTurret);

        double output = controller.calculate(turretAngle);
        assertTrue("Should command positive rotation", output > 0);

        // Robot rotates right 45°
        robotHeading = 45.0;
        desiredTurret = fieldTarget - robotHeading; // 0°
        controller.setSetPoint(desiredTurret);

        output = controller.calculate(turretAngle); // Turret still at 0°
        // Now error should be 0
        assertEquals(0.0, output, 0.01);
    }

    @Test
    public void testTurretScenario_WrapAroundTracking() {
        controller = new AnglePIDF(0.01, 0.0, 0.0, 0.0, 0.0);

        // Robot heading causes wrap-around scenario
        double fieldTarget = 10.0;
        double robotHeading = 350.0; // Robot rotated almost full circle

        // Desired turret angle wraps around
        double desiredTurret = fieldTarget - robotHeading; // 10 - 350 = -340, wraps to 20°
        controller.setSetPoint(desiredTurret);

        double turretAngle = 0.0;
        double output = controller.calculate(turretAngle);

        // Error from 0 to -340 wraps to 20°, so output should be positive
        assertTrue("Should command rotation in shortest direction", output > 0);
    }

    @Test
    public void testBasicResponse() {
        // Very simple test: controller should produce output in correct direction
        controller = new AnglePIDF(0.1, 0.0, 0.0, 0.0, 0.0);
        controller.setSetPoint(10.0);

        double output = controller.calculate(0.0);

        // With positive error (setpoint > measurement), output should be positive
        assertTrue("Controller should produce positive output for positive error, got: " + output,
                   output > 0);

        // Test negative error
        controller.setSetPoint(-10.0);
        output = controller.calculate(0.0);

        // With negative error (setpoint < measurement), output should be negative
        assertTrue("Controller should produce negative output for negative error, got: " + output,
                   output < 0);
    }
}
