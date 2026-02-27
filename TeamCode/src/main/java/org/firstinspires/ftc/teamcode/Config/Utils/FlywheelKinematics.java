package org.firstinspires.ftc.teamcode.Config.Utils;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelKinematics {

    // Regression formula coefficients: velocity = a + b*range + c*range^2 + d*range^3
    // These will be determined through testing and data analysis
    public static double COEFFICIENT_A = 0.0;  // Constant term (y-intercept)
    public static double COEFFICIENT_B = 0.0;  // Linear term
    public static double COEFFICIENT_C = 0.0;  // Quadratic term
    public static double COEFFICIENT_D = 0.0;  // Cubic term

    // Alternative: Simple linear regression for initial testing
    // velocity = intercept + slope * range
    public static double LINEAR_INTERCEPT = 800.0;  // Base velocity (ticks/sec)
    public static double LINEAR_SLOPE = 50.0;        // Velocity increase per inch of distance

    // Toggle between polynomial and linear mode for testing
    public static boolean USE_POLYNOMIAL_REGRESSION = false;

    /**
     * Calculate the target flywheel speed based on distance to target
     * @param range Distance to target in inches
     * @return Target flywheel velocity in ticks per second
     */
    public static double calculateFlywheelSpeed(double range) {
        if (USE_POLYNOMIAL_REGRESSION) {
            // Polynomial regression: velocity = a + b*range + c*range^2 + d*range^3
            return COEFFICIENT_A
                   + COEFFICIENT_B * range
                   + COEFFICIENT_C * Math.pow(range, 2)
                   + COEFFICIENT_D * Math.pow(range, 3);
        } else {
            // Simple linear regression: velocity = intercept + slope * range
            return LINEAR_INTERCEPT + LINEAR_SLOPE * range;
        }
    }

    /**
     * Calculate the target flywheel speed with custom coefficients (for testing)
     * @param range Distance to target in inches
     * @param a Constant coefficient
     * @param b Linear coefficient
     * @param c Quadratic coefficient
     * @param d Cubic coefficient
     * @return Target flywheel velocity in ticks per second
     */
    public static double calculateFlywheelSpeedCustom(double range, double a, double b, double c, double d) {
        return a + b * range + c * Math.pow(range, 2) + d * Math.pow(range, 3);
    }
}
