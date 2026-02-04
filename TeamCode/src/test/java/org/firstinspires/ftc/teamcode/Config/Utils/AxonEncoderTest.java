package org.firstinspires.ftc.teamcode.Config.Utils;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

/**
 * Unit tests for AxonEncoder class.
 * Tests encoder initialization, wrap detection, and angle calculations.
 */
public class AxonEncoderTest {

    private static final double DELTA = 0.1; // Tolerance for double comparisons
    private static final double MAX_VOLTAGE = 3.3;

    @Mock
    private AnalogInput analogInput;

    @Before
    public void setUp() {
        MockitoAnnotations.openMocks(this);
    }

    /**
     * Helper to convert degrees to voltage (0-360 maps to 0-3.3V)
     */
    private double degreesToVoltage(double degrees) {
        // Normalize to 0-360 range first
        while (degrees < 0) degrees += 360;
        while (degrees >= 360) degrees -= 360;
        return (degrees / 360.0) * MAX_VOLTAGE;
    }

    // ==================== Initialization Tests ====================

    @Test
    public void testInitialization_AtZeroDegrees() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(0.0));

        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        assertEquals(0.0, encoder.getRawAngle(), DELTA);
        assertEquals(0.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testInitialization_At180Degrees() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(180.0));

        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        assertEquals(180.0, encoder.getRawAngle(), DELTA);
        assertEquals(180.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testInitialization_WithOffset() {
        // Raw angle at 0°, with 180° offset should give 180° unwrapped
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(0.0));

        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 180.0);
        encoder.update();

        assertEquals(0.0, encoder.getRawAngle(), DELTA);
        assertEquals(180.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testInitialization_WithOffset_At180() {
        // Raw angle at 180°, with 180° offset should give 360° unwrapped
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(180.0));

        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 180.0);
        encoder.update();

        assertEquals(180.0, encoder.getRawAngle(), DELTA);
        assertEquals(360.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    // ==================== Movement Without Wrapping Tests ====================

    @Test
    public void testMovement_SmallPositiveChange() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(90.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        double initialAngle = encoder.getUnwrappedEncoderAngle();

        // Move to 100°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(100.0));
        encoder.update();

        assertEquals(initialAngle + 10.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testMovement_SmallNegativeChange() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(90.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        double initialAngle = encoder.getUnwrappedEncoderAngle();

        // Move to 80°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(80.0));
        encoder.update();

        assertEquals(initialAngle - 10.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testMovement_LargePositiveChange() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        double initialAngle = encoder.getUnwrappedEncoderAngle();

        // Move to 170° (160° change, no wrap)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(170.0));
        encoder.update();

        assertEquals(initialAngle + 160.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    // ==================== Wrap Detection Tests ====================

    @Test
    public void testWrap_PositiveWrap_CrossingZero() {
        // Start at 350°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        double initialAngle = encoder.getUnwrappedEncoderAngle();
        assertEquals(350.0, initialAngle, DELTA);

        // Move to 10° (crossed 360 -> 0, positive direction)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        encoder.update();

        // Should be 350 + 20 = 370°
        assertEquals(370.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testWrap_NegativeWrap_CrossingZero() {
        // Start at 10°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        double initialAngle = encoder.getUnwrappedEncoderAngle();
        assertEquals(10.0, initialAngle, DELTA);

        // Move to 350° (crossed 0 -> 360, negative direction)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        encoder.update();

        // Should be 10 - 20 = -10°
        assertEquals(-10.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testWrap_MultiplePositiveWraps() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        double baseAngle = encoder.getUnwrappedEncoderAngle();

        // First wrap: 350 -> 10 (should become previous + 20°)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        encoder.update();
        double afterFirstWrap = encoder.getUnwrappedEncoderAngle();
        assertEquals(baseAngle + 20.0, afterFirstWrap, DELTA); // 350 -> 10 = +20°

        // Continue to 350 (within same revolution, should be +340° from wrap point)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        encoder.update();
        assertEquals(afterFirstWrap + 340.0, encoder.getUnwrappedEncoderAngle(), DELTA);

        // Second wrap: 350 -> 10 (should be another +20°)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        encoder.update();
        assertEquals(afterFirstWrap + 340.0 + 20.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testWrap_MultipleNegativeWraps() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        double baseAngle = encoder.getUnwrappedEncoderAngle();

        // First wrap: 10 -> 350 (should become previous - 20°)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        encoder.update();
        double afterFirstWrap = encoder.getUnwrappedEncoderAngle();
        assertEquals(baseAngle - 20.0, afterFirstWrap, DELTA); // 10 -> 350 = -20°

        // Continue to 10 (within same revolution going backwards, should be -340° from wrap point)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        encoder.update();
        assertEquals(afterFirstWrap - 340.0, encoder.getUnwrappedEncoderAngle(), DELTA);

        // Second wrap: 10 -> 350 (should be another -20°)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        encoder.update();
        assertEquals(afterFirstWrap - 340.0 - 20.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    // ==================== Gear Ratio Tests ====================

    @Test
    public void testGearRatio_OutputAngle() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(180.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        // Encoder at 180°, gear ratio 2:1, output should be 90°
        assertEquals(180.0, encoder.getUnwrappedEncoderAngle(), DELTA);
        assertEquals(90.0, encoder.getUnwrappedOutputAngle(), DELTA);
    }

    @Test
    public void testGearRatio_DifferentRatios() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(180.0));

        // Test 1:1 ratio
        AxonEncoder encoder1 = new AxonEncoder(analogInput, 1.0, 0.0);
        encoder1.update();
        assertEquals(180.0, encoder1.getUnwrappedOutputAngle(), DELTA);

        // Test 3:1 ratio
        AxonEncoder encoder3 = new AxonEncoder(analogInput, 3.0, 0.0);
        encoder3.update();
        assertEquals(60.0, encoder3.getUnwrappedOutputAngle(), DELTA);

        // Test 0.5:1 ratio (output rotates faster than encoder)
        AxonEncoder encoderHalf = new AxonEncoder(analogInput, 0.5, 0.0);
        encoderHalf.update();
        assertEquals(360.0, encoderHalf.getUnwrappedOutputAngle(), DELTA);
    }

    @Test
    public void testGearRatio_WithWrapping() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        // Wrap forward
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        encoder.update();

        // Encoder at 370°, output at 185°
        assertEquals(370.0, encoder.getUnwrappedEncoderAngle(), DELTA);
        assertEquals(185.0, encoder.getUnwrappedOutputAngle(), DELTA);
    }

    // ==================== Wrapped Output Angle Tests ====================

    @Test
    public void testWrappedOutputAngle_WithinRange() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(90.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 1.0, 0.0);
        encoder.update();

        assertEquals(90.0, encoder.getWrappedOutputAngle(), DELTA);
    }

    @Test
    public void testWrappedOutputAngle_AfterWrap() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 1.0, 0.0);
        encoder.update();

        // Wrap forward to 370°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        encoder.update();

        // Unwrapped is 370°, wrapped should be 10°
        assertEquals(370.0, encoder.getUnwrappedEncoderAngle(), DELTA);
        assertEquals(10.0, encoder.getWrappedOutputAngle(), DELTA);
    }

    @Test
    public void testWrappedOutputAngle_Negative() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 1.0, 0.0);
        encoder.update();

        // Wrap backward to -10°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        encoder.update();

        // Unwrapped is -10°, wrapped should be -10°
        assertEquals(-10.0, encoder.getUnwrappedEncoderAngle(), DELTA);
        assertEquals(-10.0, encoder.getWrappedOutputAngle(), DELTA);
    }

    // ==================== Reset Tests ====================

    @Test
    public void testReset_ToZero() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(180.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        assertEquals(180.0, encoder.getUnwrappedEncoderAngle(), DELTA);

        encoder.resetUnwrappedEncoderAngle(0.0);
        assertEquals(0.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testReset_ToArbitraryValue() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(90.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        encoder.resetUnwrappedEncoderAngle(720.0);
        assertEquals(720.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testReset_ContinuesTracking() {
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(90.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        encoder.resetUnwrappedEncoderAngle(0.0);

        // Move 10 degrees
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(100.0));
        encoder.update();

        assertEquals(10.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    // ==================== Edge Cases ====================

    @Test
    public void testEdgeCase_ExactlyAt180Boundary() {
        // Test behavior exactly at the wrap detection threshold
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(180.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        // Move to 181° - should NOT trigger wrap
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(181.0));
        encoder.update();

        assertEquals(181.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testEdgeCase_JustUnderWrapThreshold() {
        // Start at 1°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(1.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        // Move to 179° - should NOT trigger wrap (delta = 178, under 180 threshold)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(179.0));
        encoder.update();

        assertEquals(179.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    @Test
    public void testEdgeCase_JustOverWrapThreshold() {
        // Start at 1°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(1.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        // Move to 359° (delta = 358, over 180 threshold, detected as -2° change)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(359.0));
        encoder.update();

        // Should detect this as moving backward 2 degrees, not forward 358
        assertEquals(-1.0, encoder.getUnwrappedEncoderAngle(), DELTA);
    }

    // ==================== Turret-Specific Integration Tests ====================

    @Test
    public void testTurretScenario_InitializationWith180Offset() {
        // Simulate turret encoder initialization as done in Turret class
        // Raw encoder at 0° with 180° offset
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(0.0));

        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 180.0);
        encoder.update();

        // Unwrapped encoder angle should be 180°
        assertEquals(180.0, encoder.getUnwrappedEncoderAngle(), DELTA);
        // Turret angle (unwrapped / gearRatio) should be 90°
        assertEquals(90.0, encoder.getUnwrappedOutputAngle(), DELTA);
    }

    @Test
    public void testTurretScenario_MovementFromCenter() {
        // Turret starts centered (raw 180° with 180° offset = 360° unwrapped)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(180.0));

        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 180.0);
        encoder.update();

        double initialUnwrapped = encoder.getUnwrappedEncoderAngle();
        assertEquals(360.0, initialUnwrapped, DELTA);

        // Turret moves 45° (servo moves 90°)
        // Raw angle: 180 + 90 = 270°
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(270.0));
        encoder.update();

        assertEquals(450.0, encoder.getUnwrappedEncoderAngle(), DELTA);
        assertEquals(225.0, encoder.getUnwrappedOutputAngle(), DELTA);
    }

    @Test
    public void testTurretScenario_MultipleFullRotations() {
        // Test turret making multiple full rotations
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(0.0));

        AxonEncoder encoder = new AxonEncoder(analogInput, 2.0, 0.0);
        encoder.update();

        // Simulate gradual movement through multiple wraps
        double[] positions = {90, 180, 270, 350, 10, 90, 180, 270, 350, 10};
        double expectedUnwrapped = 0;

        for (int i = 0; i < positions.length; i++) {
            double prevRaw = (i == 0) ? 0 : positions[i - 1];
            double currRaw = positions[i];
            double delta = currRaw - prevRaw;

            // Adjust for wrap
            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;

            expectedUnwrapped += delta;

            when(analogInput.getVoltage()).thenReturn(degreesToVoltage(currRaw));
            encoder.update();

            assertEquals("Failed at position " + i,
                    expectedUnwrapped, encoder.getUnwrappedEncoderAngle(), DELTA);
        }
    }

    @Test
    public void testSimpleWrapDebugging() {
        // Debug the wrap logic step by step
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(350.0));
        AxonEncoder encoder = new AxonEncoder(analogInput, 1.0, 0.0);
        encoder.update();

        double baseAngle = encoder.getUnwrappedEncoderAngle();
        System.out.println("Initial 350°: unwrapped = " + baseAngle);
        assertEquals(350.0, baseAngle, DELTA);

        // Move to 10° (should wrap +20)
        when(analogInput.getVoltage()).thenReturn(degreesToVoltage(10.0));
        encoder.update();

        double afterWrap = encoder.getUnwrappedEncoderAngle();
        System.out.println("After 350->10: unwrapped = " + afterWrap);
        System.out.println("Expected: " + (baseAngle + 20.0));
        assertEquals("350->10 should add 20", baseAngle + 20.0, afterWrap, DELTA);
    }
}
