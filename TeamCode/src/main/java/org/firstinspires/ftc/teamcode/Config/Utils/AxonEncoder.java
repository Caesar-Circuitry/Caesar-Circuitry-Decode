package org.firstinspires.ftc.teamcode.Config.Utils;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * AxonEncoder class for tracking absolute rotation with wrapping support
 * Handles gear ratios and provides unwrapped angle tracking (like motor encoders)
 */
public class AxonEncoder {
    private AnalogInput encoder;
    private double gearRatio;
    private double angleOffset; // Offset applied to raw angle before unwrapping (in degrees)
    private double maxVoltage;

    // Tracking variables
    private double previousRawAngle = 0;
    private double unwrappedEncoderAngle = 0; // Total accumulated encoder rotation
    private boolean initialized = false;

    /**
     * Create an AxonEncoder for tracking absolute rotation
     * @param encoder The analog input connected to the Axon encoder
     * @param gearRatio Gear ratio between encoder and output (e.g., 2.0 for 2:1 reduction)
     * @param angleOffset Offset in degrees to apply to raw encoder reading (e.g., 180 for mounting offset)
     * @param maxVoltage Maximum voltage of the encoder (typically 3.3V)
     */
    public AxonEncoder(AnalogInput encoder, double gearRatio, double angleOffset, double maxVoltage) {
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.angleOffset = angleOffset;
        this.maxVoltage = maxVoltage;
    }

    /**
     * Create an AxonEncoder with default 3.3V max voltage
     */
    public AxonEncoder(AnalogInput encoder, double gearRatio, double angleOffset) {
        this(encoder, gearRatio, angleOffset, 3.3);
    }

    /**
     * Update the encoder reading - call this in your read() method every loop
     */
    public void update() {
        // Read raw angle from encoder (0-360)
        double rawAngle = (encoder.getVoltage() / maxVoltage) * 360.0;

        if (!initialized) {
            // First reading - initialize tracking
            previousRawAngle = rawAngle;
            unwrappedEncoderAngle = rawAngle + angleOffset;
            initialized = true;
            return;
        }

        // Detect wraps and accumulate
        double delta = rawAngle - previousRawAngle;

        if (delta > 180) {
            delta -= 360; // Wrapped backwards (360 -> 0)
        } else if (delta < -180) {
            delta += 360; // Wrapped forwards (0 -> 360)
        }

        unwrappedEncoderAngle += delta;
        previousRawAngle = rawAngle;
    }

    /**
     * Get the raw encoder angle (0-360, no offset, no gear ratio)
     * @return Raw angle in degrees
     */
    public double getRawAngle() {
        return (encoder.getVoltage() / maxVoltage) * 360.0;
    }

    /**
     * Get the unwrapped encoder angle (includes offset, before gear ratio)
     * This value can be any number (positive or negative, beyond 360)
     * Similar to motor encoder ticks
     * @return Unwrapped encoder angle in degrees
     */
    public double getUnwrappedEncoderAngle() {
        return unwrappedEncoderAngle;
    }

    /**
     * Get the unwrapped output angle (after gear ratio applied)
     * This is the actual position of the mechanism being controlled
     * @return Unwrapped output angle in degrees
     */
    public double getUnwrappedOutputAngle() {
        return unwrappedEncoderAngle / gearRatio;
    }

    /**
     * Get the wrapped output angle (-180 to 180)
     * @return Wrapped output angle in degrees
     */
    public double getWrappedOutputAngle() {
        return wrap180(getUnwrappedOutputAngle());
    }

    /**
     * Wrap angle to range [-180, 180]
     * Uses while loops for consistency with TurretMath.wrap180()
     * @param angle Angle in degrees
     * @return Wrapped angle in degrees
     */
    private static double wrap180(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Reset the unwrapped encoder angle to a specific value
     * Useful for calibration or initialization
     * @param angle The angle to reset to (in encoder degrees, before gear ratio)
     */
    public void resetUnwrappedEncoderAngle(double angle) {
        unwrappedEncoderAngle = angle;
        previousRawAngle = getRawAngle();
        initialized = true;
    }

    /**
     * Get the gear ratio
     * @return The gear ratio
     */
    public double getGearRatio() {
        return gearRatio;
    }

    /**
     * Set a new gear ratio
     * @param gearRatio New gear ratio
     */
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }
}
