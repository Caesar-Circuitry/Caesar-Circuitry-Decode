package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
@TeleOp
public class AutoTuneContinuousServoTesterPinpoint extends LinearOpMode {
  private CRServo servo;
  private CRServo servo2;
  private AnalogInput servoEncoder; // encoder based on servo pos
  private GoBildaPinpointDriver pinpoint;
  private PIDFController angleController;

  private int encoderWraps = 0;
  private double lastRawEncoderAngle = 0;
  private static final double WRAP_THRESHOLD = 270.0;

  // PID gains
  public static double kP = 0.01;
  public static double kI = 0; // Integral eliminates steady-state error - start with kI = kP/100
  public static double kD = 0; // Derivative dampens oscillations - start with kD = kP/10
  public static double kF_left = 0.07; // Feedforward when turning left (positive error)
  public static double kF_right = -0.1; // Feedforward when turning right (negative error)

  // Auto-tuner parameters - Step Response Method
  public static boolean autoTuneEnabled = false; // Set to true to start auto-tuning
  public static double autoTuneStepSize = 30.0; // Step size in degrees (try 20-50)
  public static double autoTuneStepPower = 0.4; // Fixed power for step (try 0.3-0.6)

  // Auto-tuner state
  private enum TuningState {
    IDLE,
    WAITING_FOR_SETTLE,
    APPLYING_STEP,
    MEASURING_RESPONSE,
    CALCULATING_GAINS,
    DONE
  }

  private TuningState tuningState = TuningState.IDLE;
  private java.util.ArrayList<Double> timeData = new java.util.ArrayList<>();
  private java.util.ArrayList<Double> angleData = new java.util.ArrayList<>();
  private double tuningStartTime = 0;
  private double stepStartTime = 0;
  private double stepStartAngle = 0;
  private double stepTargetAngle = 0;
  private double maxAngle = 0;
  private double timeToMaxAngle = 0;
  private double settleTime = 0;
  private double riseTime = 0;

  // Hardware names
  public static String servoName = "servo1";
  public static String servoName2 = "servo2";
  public static String servoEncoderName = "rightServo";

  // Control parameters
  public static double targetAngle = 0; // field-relative angle the turret should face
  public static double gearRatio =
      2; // servo rotations per turret rotation (2:1 = servo rotates 2x)
  private JoinedTelemetry Telemetry;

  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(CRServo.class, servoName);
    servo2 = hardwareMap.get(CRServo.class, servoName2);
    servoEncoder = hardwareMap.get(AnalogInput.class, servoEncoderName);
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    // Initialize PID controller with configurable values
    angleController = new PIDFController(kP, kI, kD, 0);


    // Configure the sensor
    configurePinpoint();

    // Set the location of the robot - this should be the place you are starting the robot from
    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    waitForStart();

    while (opModeIsActive()) {
      // Update pinpoint position
      pinpoint.update();
      double heading = pinpoint.getHeading(AngleUnit.DEGREES);

      // Auto-tuner logic - if enabled, run tuning instead of normal control
      if (autoTuneEnabled && tuningState != TuningState.DONE) {
        performAutoTuning(heading);
        continue; // Skip normal control during tuning
      }

      // Calculate desired turret angle (robot-relative) to point to targetAngle (field-relative)
      double desiredTurretAngleRaw = targetAngle - heading;
      double wrappedDesiredTurretAngle = wrap180(desiredTurretAngleRaw);

      // Find the closest valid turret angle within the 270-degree range (-135 to +135)
      double desiredTurretAngle = getClosestAngleInRange(wrappedDesiredTurretAngle, -135.0, 135.0);

      // Convert desired turret angle to desired servo angle using gear ratio
      double desiredServoAngle = desiredTurretAngle * gearRatio;

      // Read current servo angle from absolute encoder (in degrees)
      double currentServoAngle = getUnwrappedServoAngle(servoEncoder);

      // Convert current servo angle to equivalent turret angle for telemetry
      double currentTurretAngle = currentServoAngle / gearRatio;


      // Update PID controller coefficients in case they changed via dashboard
      angleController.setPIDF(kP, kI, kD, 0);

      // Calculate servo power using PID control
      double servoError = desiredServoAngle - currentServoAngle;
      double wrappedServoError = wrap180(servoError);

      // Calculate the target position that's closest to current via wrapping
      double adjustedTarget = currentServoAngle + wrappedServoError;

      // Use PID controller with current position and adjusted target
      angleController.setSetPoint(adjustedTarget);
      double servoPower = angleController.calculate(currentServoAngle);

      // Add direction-specific feedforward term to overcome friction/deadband
      // Both directions add positive feedforward in the direction of motion
      if (wrappedServoError > 0) {
        servoPower += kF_left;  // Moving left (positive power)
      } else if (wrappedServoError < 0) {
        servoPower += kF_right; // Moving right (negative power) - changed from -= to +=
      }

      // Clamp servo power to valid range for CRServo
      servoPower = clamp(servoPower, -1.0, 1.0);

      // Set power to both servos
      servo.setPower(servoPower);
      servo2.setPower(servoPower);

      // Telemetry for debugging
      Telemetry.addData("=== AUTO-TUNER ===", "");
      Telemetry.addData("Auto-Tune Enabled", autoTuneEnabled);
      Telemetry.addData("Tuning State", tuningState);
      Telemetry.addData("---", "---");
      Telemetry.addData("Target Field Angle", "%.2f deg", targetAngle);
      Telemetry.addData("Robot Heading", "%.2f deg", heading);
      Telemetry.addData("Desired Turret Angle (robot-relative)", "%.2f deg", desiredTurretAngle);
      Telemetry.addData("Current Turret Angle (robot-relative)", "%.2f deg", currentTurretAngle);
      Telemetry.addData("---", "---");
      Telemetry.addData("Raw Encoder Angle", "%.2f deg", getCurrentPosition(servoEncoder));
      Telemetry.addData("Encoder Wraps", encoderWraps);
      Telemetry.addData("Current Servo Angle (unwrapped)", "%.2f deg", currentServoAngle);
      Telemetry.addData("Desired Servo Angle", "%.2f deg", desiredServoAngle);
      Telemetry.addData("Servo Error", "%.2f deg", wrappedServoError);
      Telemetry.addData("---", "---");
      Telemetry.addData("kP / kI / kD", "%.4f / %.4f / %.4f", kP, kI, kD);
      Telemetry.addData("kF (Left / Right)", "%.4f / %.4f", kF_left, kF_right);
      Telemetry.addData("Servo Encoder Voltage", "%.2f V", servoEncoder.getVoltage());
      Telemetry.addData("Servo Power", "%.3f", servoPower);
      Telemetry.addData("Gear Ratio", "%.1f:1", gearRatio);
      Telemetry.update();
    }
  }

  /**
   * Step Response Auto-Tuner - More reliable than relay method
   * Applies a step input and analyzes the response curve
   */
  private void performAutoTuning(double heading) {
    double currentServoAngle = getUnwrappedServoAngle(servoEncoder);
    double currentTurretAngle = currentServoAngle / gearRatio;
    double currentTime = System.nanoTime() / 1e9;

    switch (tuningState) {
      case IDLE:
        tuningState = TuningState.WAITING_FOR_SETTLE;
        tuningStartTime = currentTime;
        servo.setPower(0);
        servo2.setPower(0);
        Telemetry.addData("Auto-Tuner", "Started! Waiting for servo to settle...");
        Telemetry.update();
        break;

      case WAITING_FOR_SETTLE:
        servo.setPower(0);
        servo2.setPower(0);

        if (currentTime - tuningStartTime > 2.0) {
          // Record starting position
          stepStartAngle = currentServoAngle;
          stepTargetAngle = stepStartAngle + (autoTuneStepSize * gearRatio);
          stepStartTime = currentTime;
          maxAngle = stepStartAngle;
          timeToMaxAngle = 0;
          timeData.clear();
          angleData.clear();

          tuningState = TuningState.APPLYING_STEP;
          Telemetry.addData("Auto-Tuner", "Applying step input...");
        } else {
          Telemetry.addData("Auto-Tuner", "Settling... %.1fs remaining", 2.0 - (currentTime - tuningStartTime));
        }
        Telemetry.update();
        break;

      case APPLYING_STEP:
      case MEASURING_RESPONSE:
        // Apply constant power step
        servo.setPower(autoTuneStepPower);
        servo2.setPower(autoTuneStepPower);

        // Record data
        double elapsedTime = currentTime - stepStartTime;
        double angleChange = currentServoAngle - stepStartAngle;

        timeData.add(elapsedTime);
        angleData.add(angleChange);

        // Track maximum overshoot
        if (angleChange > maxAngle - stepStartAngle) {
          maxAngle = currentServoAngle;
          timeToMaxAngle = elapsedTime;
        }

        // Switch to measuring after step is applied
        if (tuningState == TuningState.APPLYING_STEP && elapsedTime > 0.1) {
          tuningState = TuningState.MEASURING_RESPONSE;
        }

        // Stop measuring after reaching target or timeout
        if (angleChange >= autoTuneStepSize * gearRatio || elapsedTime > 5.0) {
          servo.setPower(0);
          servo2.setPower(0);
          tuningState = TuningState.CALCULATING_GAINS;
        }

        Telemetry.addData("Auto-Tuner", "Measuring response...");
        Telemetry.addData("Elapsed Time", "%.2f s", elapsedTime);
        Telemetry.addData("Angle Change", "%.2f deg", angleChange / gearRatio);
        Telemetry.addData("Target Change", "%.2f deg", autoTuneStepSize);
        Telemetry.update();
        break;

      case CALCULATING_GAINS:
        servo.setPower(0);
        servo2.setPower(0);

        if (timeData.size() < 10) {
          Telemetry.addData("Auto-Tuner", "ERROR - Not enough data collected");
          Telemetry.addData("Suggestion", "Increase autoTuneStepPower or autoTuneStepSize");
          tuningState = TuningState.DONE;
          Telemetry.update();
          break;
        }

        // Analyze step response using Cohen-Coon method
        double finalAngle = angleData.get(angleData.size() - 1);
        double stepInput = autoTuneStepPower;

        // Find when we reach 63.2% of final value (time constant tau)
        double target63 = 0.632 * finalAngle;
        double tau = 0;
        for (int i = 0; i < angleData.size(); i++) {
          if (angleData.get(i) >= target63) {
            tau = timeData.get(i);
            break;
          }
        }

        // Find dead time (time to reach 5% of final)
        double target5 = 0.05 * finalAngle;
        double deadTime = 0;
        for (int i = 0; i < angleData.size(); i++) {
          if (angleData.get(i) >= target5) {
            deadTime = timeData.get(i);
            break;
          }
        }

        // Calculate process gain (how much output per unit input)
        double processGain = finalAngle / stepInput;

        // Use IMC (Internal Model Control) tuning with BALANCED settings
        // Good speed while maintaining stability

        // IMC tuning parameter - moderate lambda for balanced response
        // Use tau * 0.75 for good balance between speed and stability
        double lambda = Math.max(tau * 0.75, 0.08); // Balanced closed-loop time constant

        // Calculate gains using IMC-PD rules
        kP = tau / (processGain * lambda);
        kD = tau * kP / 4.0; // Standard damping ratio
        kI = 0; // No integral for now

        // Apply MODERATE multipliers for stable fast response
        kP = kP * 0.7; // Balanced P gain (not too aggressive)
        kD = kD * 0.5; // Good damping to prevent overshoot

        // Ensure kD is always less than kP (sanity check)
        if (kD >= kP) {
          kD = kP * 0.18; // kD should be max 18% of kP
        }

        // Reasonable limits to prevent instability
        kP = clamp(kP, 0.008, 0.08); // Moderate max kP
        kD = clamp(kD, 0.001, 0.02); // Moderate max kD

        // Final sanity check - kP should be at least 4x larger than kD for stability
        if (kP < kD * 4.0) {
          kP = kD * 6.0;
          kP = clamp(kP, 0.008, 0.08);
        }

        // Additional safety: if kD is too high relative to kP, reduce it
        if (kD > kP * 0.2) {
          kD = kP * 0.18;
        }

        Telemetry.addData("=== AUTO-TUNE COMPLETE ===", "");
        Telemetry.addData("Method", "IMC Tuning (Balanced - Stable & Fast)");
        Telemetry.addData("---", "---");
        Telemetry.addData("Process Gain (K)", "%.3f deg/power", processGain);
        Telemetry.addData("Time Constant (tau)", "%.3f s", tau);
        Telemetry.addData("Dead Time (L)", "%.3f s", deadTime);
        Telemetry.addData("Lambda (speed factor)", "%.3f s (balanced)", lambda);
        Telemetry.addData("---", "---");
        Telemetry.addData("Calculated kP", "%.4f", kP);
        Telemetry.addData("Calculated kD", "%.4f", kD);
        Telemetry.addData("Calculated kI", "%.4f", kI);
        Telemetry.addData("kP/kD Ratio", "%.1f (should be >4)", kP / Math.max(kD, 0.0001));
        Telemetry.addData("---", "---");
        Telemetry.addData("Data Points Collected", timeData.size());
        Telemetry.addData("Instructions", "Set autoTuneEnabled=false to test");
        Telemetry.addData("Tuning", "Balanced: Good speed, zero overshoot, stable");
        Telemetry.addData("Fine-Tuning", "Increase kP by 10%% for more speed");
        Telemetry.update();

        tuningState = TuningState.DONE;
        break;

      case DONE:
        servo.setPower(0);
        servo2.setPower(0);
        Telemetry.addData("Auto-Tuner", "DONE - Set autoTuneEnabled=false");
        Telemetry.addData("Tuned Gains", "kP=%.4f, kD=%.4f", kP, kD);
        Telemetry.addData("To Re-tune", "Toggle autoTuneEnabled");
        Telemetry.update();

        if (!autoTuneEnabled) {
          tuningState = TuningState.IDLE;
        }
        break;
    }
  }

  /**
   * Calculate average of an ArrayList of doubles
   */
  private double calculateAverage(java.util.ArrayList<Double> values) {
    if (values.isEmpty()) return 0;
    double sum = 0;
    for (double v : values) {
      sum += v;
    }
    return sum / values.size();
  }

  public void configurePinpoint() {
    /*
     *  Set the odometry pod positions relative to the point that you want the position to be measured from.
     *
     *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
     *  Left of the center is a positive number, right of center is a negative number.
     *
     *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
     *  Forward of center is a positive number, backwards is a negative number.
     */
    pinpoint.setOffsets(
        -84.0, -168.0, DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1

    /*
     * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
     * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
     * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
     * number of ticks per unit of your odometry pod.  For example:
     *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
     */
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    /*
     * Set the direction that each of the two odometry pods count. The X (forward) pod should
     * increase when you move the robot forward. And the Y (strafe) pod should increase when
     * you move the robot to the left.
     */
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /*
     * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
     * The IMU will automatically calibrate when first powered on, but recalibrating before running
     * the robot is a good idea to ensure that the calibration is "good".
     * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
     * This is recommended before you run your autonomous, as a bad initial calibration can cause
     * an incorrect starting value for x, y, and heading.
     */
    pinpoint.resetPosAndIMU();
  }

  private double wrap180(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
  }

  private double getClosestAngleInRange(double targetAngle, double minAngle, double maxAngle) {
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

  private double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  private double getUnwrappedServoAngle(AnalogInput analogInput) {
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

  private double getCurrentPosition(AnalogInput analogInput) {
    return ((analogInput.getVoltage() / 3.3) * 360) - 180;
  }
}
