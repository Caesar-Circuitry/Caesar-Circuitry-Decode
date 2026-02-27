package org.firstinspires.ftc.teamcode.Config.Utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * ThroughBoreEncoder class for tracking absolute rotation using a REV Through Bore encoder.
 * The encoder is read via a DcMotorEx port using tick counting.
 * Handles gear ratios and provides unwrapped angle tracking (like motor encoders).
 *
 * REV Through Bore: 8192 ticks per revolution
 */
public class ThroughBoreEncoder {
    private DcMotorEx encoderMotor;
    private double gearRatio;
    private double ticksPerRevolution;

    // Tracking variables
    private int zeroTicks = 0;
    private double unwrappedEncoderAngle = 0; // Total accumulated encoder rotation in degrees

    /**
     * Create a ThroughBoreEncoder for tracking absolute rotation
     * @param encoderMotor The DcMotorEx whose encoder port the Through Bore is connected to
     * @param gearRatio Gear ratio between encoder and output (e.g., 2.0 for 2:1 reduction)
     * @param ticksPerRevolution Ticks per full encoder revolution (8192 for REV Through Bore)
     * @param resetOnInit If true, reset encoder ticks to 0 on initialization
     */
    public ThroughBoreEncoder(DcMotorEx encoderMotor, double gearRatio, double ticksPerRevolution, boolean resetOnInit) {
        this.encoderMotor = encoderMotor;
        this.gearRatio = gearRatio;
        this.ticksPerRevolution = ticksPerRevolution;

        if (resetOnInit) {
            encoderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        zeroTicks = encoderMotor.getCurrentPosition();
        unwrappedEncoderAngle = 0;
    }

    /**
     * Create a ThroughBoreEncoder with default 8192 ticks/rev and reset on init
     */
    public ThroughBoreEncoder(DcMotorEx encoderMotor, double gearRatio) {
        this(encoderMotor, gearRatio, 8192.0, true);
    }

    /**
     * Update the encoder reading - call this in your read() method every loop.
     * For a Through Bore encoder on DcMotorEx, the tick counting is handled
     * by the hardware, so we just convert ticks to degrees.
     */
    public void update() {
        int ticks = encoderMotor.getCurrentPosition() - zeroTicks;
        unwrappedEncoderAngle = ticks * (360.0 / ticksPerRevolution);
    }

    /**
     * Get the raw tick count from the encoder (relative to zero)
     * @return Raw ticks
     */
    public int getRawTicks() {
        return encoderMotor.getCurrentPosition() - zeroTicks;
    }

    /**
     * Get the raw encoder angle (0-360, no gear ratio)
     * @return Raw angle in degrees, wrapped to [0, 360)
     */
    public double getRawAngle() {
        double angle = unwrappedEncoderAngle % 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    /**
     * Get the unwrapped encoder angle (before gear ratio).
     * This value can be any number (positive or negative, beyond 360).
     * Similar to motor encoder ticks but in degrees.
     * @return Unwrapped encoder angle in degrees
     */
    public double getUnwrappedEncoderAngle() {
        return unwrappedEncoderAngle;
    }

    /**
     * Get the unwrapped output angle (after gear ratio applied).
     * This is the actual position of the mechanism being controlled.
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
     * Reset the encoder zero point to the current position.
     * After calling this, the unwrapped angle will read 0.
     */
    public void zeroNow() {
        zeroTicks = encoderMotor.getCurrentPosition();
        unwrappedEncoderAngle = 0;
    }

    /**
     * Reset the unwrapped encoder angle to a specific value.
     * Useful for calibration or initialization.
     * @param angle The angle to reset to (in encoder degrees, before gear ratio)
     */
    public void resetUnwrappedEncoderAngle(double angle) {
        zeroTicks = encoderMotor.getCurrentPosition();
        unwrappedEncoderAngle = angle;
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

    /**
     * Wrap angle to range [-180, 180]
     * @param angle Angle in degrees
     * @return Wrapped angle in degrees
     */
    private static double wrap180(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }
}

