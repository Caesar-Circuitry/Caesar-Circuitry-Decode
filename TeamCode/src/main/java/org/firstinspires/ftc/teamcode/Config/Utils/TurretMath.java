package org.firstinspires.ftc.teamcode.Config.Utils;

import static org.firstinspires.ftc.teamcode.Config.Constants.Turret.WRAP_THRESHOLD;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class TurretMath {

    private static int encoderWraps = 0;
    private static double lastRawEncoderAngle = 0;

    public static double wrap180(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    public static double getClosestAngleInRange(double targetAngle, double minAngle, double maxAngle) {
        // If target is within range, use it directly
        if (targetAngle >= minAngle && targetAngle <= maxAngle) {
            return targetAngle;
        }

        // Target is out of range, find the closest boundary
        // Consider wrap-around: the target might be closer via the other side of the circle

        // Calculate distance to each boundary
        double distToMin = Math.abs(wrap180(targetAngle - minAngle));
        double distToMax = Math.abs(wrap180(targetAngle - maxAngle));

        // Also consider wrapping to the opposite side
        double wrappedTarget = targetAngle > 0 ? targetAngle - 360.0 : targetAngle + 360.0;
        double distToMinWrapped = Math.abs(wrap180(wrappedTarget - minAngle));
        double distToMaxWrapped = Math.abs(wrap180(wrappedTarget - maxAngle));

        // Find minimum distance
        double minDist =
                Math.min(Math.min(distToMin, distToMax), Math.min(distToMinWrapped, distToMaxWrapped));

        // Return the boundary with minimum distance
        if (minDist == distToMin || minDist == distToMinWrapped) {
            return minAngle;
        } else {
            return maxAngle;
        }
    }

    public static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    public static double getUnwrappedServoAngle(AnalogInput analogInput) {
        double rawAngle = getCurrentPosition(analogInput);

        // Detect wrap-around
        double angleDelta = rawAngle - lastRawEncoderAngle;

        // If we jumped from near +180 to near -180 (positive wrap)
        if (angleDelta < -WRAP_THRESHOLD) {
            encoderWraps++;
        }
        // If we jumped from near -180 to near +180 (negative wrap)
        else if (angleDelta > WRAP_THRESHOLD) {
            encoderWraps--;
        }

        lastRawEncoderAngle = rawAngle;

        // Calculate unwrapped angle
        return rawAngle + (encoderWraps * 360.0);
    }

    public static double getCurrentPosition(AnalogInput analogInput) {
        return ((analogInput.getVoltage() / 3.3) * 360) - 180;
    }

    /**
     * Determines the safe turret angle to target, preventing wire damage.
     * If the desired angle is outside the valid range (-135 to +135), this method
     * intelligently chooses which limit to snap to based on the current position,
     * ensuring the turret never tries to wrap around through the forbidden zone.
     *
     * @param desiredAngle The desired turret angle (wrapped to -180 to +180)
     * @param currentAngle The current turret angle (wrapped to -180 to +180)
     * @return The safe angle to target within the valid range
     */
    public static double getSafeTurretAngle(double desiredAngle, double currentAngle) {
        final double MIN_ANGLE = -135.0;
        final double MAX_ANGLE = 135.0;

        // If desired angle is within valid range, use it directly
        if (desiredAngle >= MIN_ANGLE && desiredAngle <= MAX_ANGLE) {
            return desiredAngle;
        }

        // Desired angle is outside the valid range

        // If desired angle is beyond +135 (in the range +135 to +180)
        if (desiredAngle > MAX_ANGLE) {
            // Check if we can reach +135 from current position without crossing forbidden zone
            // If current is in valid range, we can always reach +135
            // The forbidden zone is from +135 to -135 going through +180/-180
            return MAX_ANGLE;
        }

        // If desired angle is beyond -135 (in the range -180 to -135)
        if (desiredAngle < MIN_ANGLE) {
            // Check if we can reach -135 from current position without crossing forbidden zone
            // If current is in valid range, we can always reach -135
            // The forbidden zone is from -135 to +135 going through -180/+180
            return MIN_ANGLE;
        }

        // Fallback: shouldn't reach here due to wrap180, but just in case
        return clamp(desiredAngle, MIN_ANGLE, MAX_ANGLE);
    }

    /**
     * Check if the path from current to target servo angle is safe.
     * A path is safe if:
     * 1. The target turret angle (wrapped) is within -135° to +135° WITH SAFETY MARGIN
     * 2. The path doesn't pass through the forbidden zone at ANY point along the journey
     * 3. We densely sample the entire path, not just wrap boundaries
     *
     * Note: The servo angles passed in already have the 180° offset baked in, so we just
     * divide by gear ratio without any offset adjustment.
     *
     * @param currentServo Current unwrapped servo angle (with 180° offset already applied)
     * @param targetServo Target unwrapped servo angle (with 180° offset already applied)
     * @param gearRatio The gear ratio between servo and turret
     * @return true if the path is safe and target is valid
     */
    public static boolean isPathSafe(double currentServo, double targetServo, double gearRatio) {
        final double MIN_TURRET = -135.0;
        final double MAX_TURRET = 135.0;
        final double SAFETY_MARGIN = 5.0; // Degrees of safety margin to prevent overshoot

        // Convert target to turret angle (offset is already in servo angle, just divide)
        double targetTurret = targetServo / gearRatio;
        double wrappedTargetTurret = wrap180(targetTurret);

        // First check: target must be within valid range WITH SAFETY MARGIN to prevent overshoot
        if (wrappedTargetTurret < (MIN_TURRET + SAFETY_MARGIN) ||
            wrappedTargetTurret > (MAX_TURRET - SAFETY_MARGIN)) {
            return false;
        }

        // Check start position (also with safety margin)
        double currentTurret = currentServo / gearRatio;
        double wrappedCurrentTurret = wrap180(currentTurret);
        if (wrappedCurrentTurret < (MIN_TURRET + SAFETY_MARGIN) ||
            wrappedCurrentTurret > (MAX_TURRET - SAFETY_MARGIN)) {
            return false;
        }

        // Sample the ENTIRE path densely to ensure we never enter forbidden zone
        // We need to check every point along the unwrapped servo path
        double servoDistance = Math.abs(targetServo - currentServo);

        // Sample at least every 5 degrees of servo motion (2.5° of turret motion with 2:1 ratio)
        int numSamples = Math.max(100, (int)(servoDistance / 5.0));

        for (int i = 0; i <= numSamples; i++) {
            double t = i / (double) numSamples;
            double sampleServo = currentServo + t * (targetServo - currentServo);
            double sampleTurret = sampleServo / gearRatio;
            double wrappedSampleTurret = wrap180(sampleTurret);

            // If any point along the path is in the forbidden zone (with margin), reject this path
            if (wrappedSampleTurret < (MIN_TURRET + SAFETY_MARGIN) ||
                wrappedSampleTurret > (MAX_TURRET - SAFETY_MARGIN)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Find the closest servo angle target from candidates around the safe angle
     * Selects the wrap that requires the shortest movement from the current position
     *
     * @param currentServoAngle Current unwrapped servo angle
     * @param targetServoAngleSafe Safe target servo angle (wrapped to one revolution)
     * @return The unwrapped servo angle target that's closest to current position
     */
    public static double getClosestServoTarget(double currentServoAngle, double targetServoAngleSafe) {
        // Find which 360° wrap we're currently in
        int currentWrapCount = (int) Math.round(currentServoAngle / 360.0);

        // Generate 3 candidates: one wrap behind, current wrap, one wrap ahead
        double[] candidates = {
            targetServoAngleSafe + ((currentWrapCount - 1) * 360.0),
            targetServoAngleSafe + (currentWrapCount * 360.0),
            targetServoAngleSafe + ((currentWrapCount + 1) * 360.0)
        };

        // Find the candidate with minimum distance
        double targetServoAngle = candidates[1]; // default to middle
        double minDistance = Double.MAX_VALUE;

        for (double candidate : candidates) {
            double distance = Math.abs(candidate - currentServoAngle);
            if (distance < minDistance) {
                minDistance = distance;
                targetServoAngle = candidate;
            }
        }

        return targetServoAngle;
    }
}
