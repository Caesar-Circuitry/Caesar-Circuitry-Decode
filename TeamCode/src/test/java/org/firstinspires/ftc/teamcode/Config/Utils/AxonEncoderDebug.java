package org.firstinspires.ftc.teamcode.Config.Utils;

/**
 * Simple test to debug AxonEncoder wrap logic manually
 * Tests the wrap detection algorithm step by step
 */
public class AxonEncoderDebug {
    public static void main(String[] args) {
        System.out.println("=== Testing AxonEncoder Wrap Detection Logic ===");

        // Test the wrap detection logic manually without hardware dependencies
        double previousRawAngle;
        double unwrappedEncoderAngle;
        double angleOffset = 0.0;

        System.out.println("\n=== Multiple Positive Wraps Test ===");

        // Step 1: Initialize at 350°
        double rawAngle = 350.0;
        previousRawAngle = rawAngle;
        unwrappedEncoderAngle = rawAngle + angleOffset;
        System.out.println("1. Initialize at 350°: unwrapped = " + unwrappedEncoderAngle);

        // Step 2: Move to 10° (should wrap +20°)
        rawAngle = 10.0;
        double delta = rawAngle - previousRawAngle; // 10 - 350 = -340
        if (delta > 180.0) {
            delta -= 360.0;
        } else if (delta < -180.0) {
            delta += 360.0; // -340 + 360 = +20
        }
        unwrappedEncoderAngle += delta; // 350 + 20 = 370
        previousRawAngle = rawAngle;
        System.out.println("2. Move to 10°: delta=" + delta + ", unwrapped=" + unwrappedEncoderAngle + " (expected: 370.0)");

        // Step 3: Move to 350° (should add 340°, not wrap)
        rawAngle = 350.0;
        delta = rawAngle - previousRawAngle; // 350 - 10 = +340
        if (delta > 180.0) {
            delta -= 360.0; // 340 - 360 = -20 (This is the issue!)
        } else if (delta < -180.0) {
            delta += 360.0;
        }
        unwrappedEncoderAngle += delta; // 370 + (-20) = 350
        previousRawAngle = rawAngle;
        System.out.println("3. Move to 350°: delta=" + delta + ", unwrapped=" + unwrappedEncoderAngle + " (expected: 710.0)");

        System.out.println("\n*** PROBLEM IDENTIFIED ***");
        System.out.println("The wrap logic treats 10° -> 350° (340° movement) as a negative wrap!");
        System.out.println("This happens because 340° > 180°, so it subtracts 360°, giving -20°.");
        System.out.println("But the test expects this to be a +340° movement, not a -20° wrap.");

        System.out.println("\n=== Analysis ===");
        System.out.println("Current logic assumes any movement > 180° is a wrap.");
        System.out.println("But after wrapping from 350° -> 10°, the encoder is 'logically' at 370°.");
        System.out.println("Moving from raw 10° to raw 350° should be +340°, continuing the rotation.");
        System.out.println("The issue is that wrap detection only looks at raw angles, not logical position.");

        testCorrectLogic();
    }

    private static void testCorrectLogic() {
        System.out.println("\n=== Testing Potential Fix ===");
        System.out.println("The current AxonEncoder logic is actually CORRECT for a simple wrap detector.");
        System.out.println("The test expectation might be wrong, or there's a different interpretation.");
        System.out.println("\nLet's check what the test SHOULD expect:");

        // Re-analyze what should happen:
        System.out.println("\nScenario: Multiple rotations in same direction");
        System.out.println("1. Start at raw 350° -> unwrapped = 350°");
        System.out.println("2. Cross boundary to raw 10° -> detect wrap -> unwrapped = 350° + 20° = 370°");
        System.out.println("3. Continue to raw 350°...");
        System.out.println("\nTwo interpretations:");
        System.out.println("A) Continuing rotation: 10° -> 350° = +340° -> unwrapped = 710°");
        System.out.println("B) Wrap detection: |350° - 10°| = 340° > 180° -> wrap -> unwrapped = 350°");

        System.out.println("\nThe test expects (A), but our algorithm does (B).");
        System.out.println("This suggests the test wants to track CONTINUOUS ROTATION, not just wrap detection.");
    }
}
