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
}
