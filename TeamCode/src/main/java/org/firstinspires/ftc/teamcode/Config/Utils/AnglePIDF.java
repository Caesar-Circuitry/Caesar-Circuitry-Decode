package org.firstinspires.ftc.teamcode.Config.Utils;

/**
 * AnglePIDF Controller
 * Handles PIDF control for angles with proper wrapping around -180 to 180 degrees
 * Designed to work with unwrapped angles (like from AxonEncoder) and provide
 * proper error calculation that respects angle wrapping
 */
public class AnglePIDF {
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF_left = 0.0;
    private double kF_right = 0.0;

    private double setPoint = 0.0;
    private double lastError = 0.0;
    private double integral = 0.0;
    private double integralMax = 1.0;

    // Danger zone limiting
    private double dangerZoneMin = -135;  // Minimum safe angle
    private double dangerZoneMax = 135.0;   // Maximum safe angle
    private boolean dangerZoneLimitEnabled = false;

    private boolean firstRun = true;

    public AnglePIDF(double kP, double kI, double kD, double kF_left, double kF_right) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF_left = kF_left;
        this.kF_right = kF_right;
    }

    /**
     * Set PIDF coefficients
     */
    public void setCoefficients(double kP, double kI, double kD, double kF_left, double kF_right) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF_left = kF_left;
        this.kF_right = kF_right;
    }

    /**
     * Set the target angle setpoint
     * @param angle Target angle in degrees (can be any value, wrapping handled internally)
     */
    public void setSetPoint(double angle) {
        this.setPoint = angle;
    }

    /**
     * Get the current setpoint
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Calculate the wrapped error between target and current angle
     * This ensures the error is always in the range [-180, 180]
     * @param setPoint Target angle in degrees
     * @param measurement Current angle in degrees
     * @return Wrapped error in degrees
     */
    private double calculateWrappedError(double setPoint, double measurement) {
        double error = setPoint - measurement;

        // Wrap error to [-180, 180] range
        while (error > 180.0) {
            error -= 360.0;
        }
        while (error < -180.0) {
            error += 360.0;
        }

        return error;
    }

    /**
     * Calculate PIDF output
     * @param measurement Current angle in degrees (can be any value, wrapping handled internally)
     * @return Power output from -1.0 to 1.0
     */
    public double calculate(double measurement) {
        // Determine effective setpoint (may be adjusted to avoid danger zone)
        double effectiveSetPoint = setPoint;

        // If danger zone limiting is enabled, check if path would enter danger zone
        if (dangerZoneLimitEnabled && pathEntersDangerZone(measurement, setPoint)) {
            // Path enters danger zone - take the long way around
            // Add 360 degrees to go the opposite direction
            if (setPoint > measurement) {
                effectiveSetPoint = setPoint - 360.0;
            } else {
                effectiveSetPoint = setPoint + 360.0;
            }
        }

        // Calculate wrapped error using effective setpoint
        double error = calculateWrappedError(effectiveSetPoint, measurement);

        // Calculate derivative (rate of change of error)
        double derivative = 0.0;
        if (!firstRun) {
            derivative = error - lastError;

            // Handle derivative discontinuity when error wraps
            if (Math.abs(derivative) > 180.0) {
                // Wrap the derivative as well
                if (derivative > 0) {
                    derivative -= 360.0;
                } else {
                    derivative += 360.0;
                }
            }
        }
        lastError = error;

        // Calculate integral with anti-windup
        integral += error;
        if (Math.abs(integral) > integralMax) {
            integral = Math.copySign(integralMax, integral);
        }

        // Calculate PIDF output with base PID
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Add directional feedforward
        if (error > 0) {
            output += kF_left;  // Moving in positive direction
        } else if (error < 0) {
            output += kF_right; // Moving in negative direction
        }

        firstRun = false;
        return output;
    }

    /**
     * Reset the controller state
     */
    public void reset() {
        lastError = 0.0;
        integral = 0.0;
        firstRun = true;
    }

    /**
     * Set integral max for anti-windup
     */
    public void setIntegralMax(double max) {
        this.integralMax = max;
    }

    /**
     * Get the last calculated error
     */
    public double getLastError() {
        return lastError;
    }

    /**
     * Get the current integral accumulation
     */
    public double getIntegral() {
        return integral;
    }

    /**
     * Enable danger zone limiting
     * When enabled, the controller will refuse to move into the danger zone
     * If a movement would enter the danger zone, it instead reverses direction
     * and takes the long way around through zero
     */
    public void enableDangerZoneLimiting() {
        this.dangerZoneLimitEnabled = true;
    }

    /**
     * Disable danger zone limiting
     */
    public void disableDangerZoneLimiting() {
        this.dangerZoneLimitEnabled = false;
    }

    /**
     * Set the danger zone boundaries
     * The controller will avoid moving into angles between dangerMin and dangerMax
     * @param dangerMin Minimum angle of danger zone (in degrees)
     * @param dangerMax Maximum angle of danger zone (in degrees)
     */
    public void setDangerZone(double dangerMin, double dangerMax) {
        this.dangerZoneMin = wrapTo180(dangerMin);
        this.dangerZoneMax = wrapTo180(dangerMax);
    }

    /**
     * Check if an angle is in the danger zone
     * @param angle Angle to check (in unwrapped degrees)
     * @return True if angle is in danger zone
     */
    private boolean isInDangerZone(double angle) {
        double wrapped = wrapTo180(angle);

        // Handle case where danger zone doesn't cross -180/180 boundary
        if (dangerZoneMin <= dangerZoneMax) {
            return wrapped >= dangerZoneMin && wrapped <= dangerZoneMax;
        }
        // Handle case where danger zone crosses -180/180 boundary (e.g., -170 to +170)
        else {
            return wrapped >= dangerZoneMin || wrapped <= dangerZoneMax;
        }
    }

    /**
     * Wrap angle to [-180, 180] range
     */
    private static double wrapTo180(double angle) {
        double wrapped = angle % 360.0;
        if (wrapped <= -180.0) {
            wrapped += 360.0;
        }
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        }
        return wrapped;
    }

    /**
     * Check if movement from current position to target would enter danger zone
     * @param currentAngle Current angle (unwrapped)
     * @param targetAngle Target angle (unwrapped)
     * @return True if path would enter danger zone
     */
    private boolean pathEntersDangerZone(double currentAngle, double targetAngle) {
        double current = wrapTo180(currentAngle);
        double target = wrapTo180(targetAngle);

        // Calculate the short path error
        double shortPathError = target - current;
        if (shortPathError > 180.0) {
            shortPathError -= 360.0;
        } else if (shortPathError < -180.0) {
            shortPathError += 360.0;
        }

        // Check intermediate points along the short path
        int steps = Math.max(10, (int) Math.abs(shortPathError));
        for (int i = 1; i < steps; i++) {
            double progress = (double) i / steps;
            double intermediateAngle = current + (shortPathError * progress);
            if (isInDangerZone(intermediateAngle)) {
                return true;
            }
        }

        return false;
    }
}
