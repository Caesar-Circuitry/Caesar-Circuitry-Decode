package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.firstinspires.ftc.teamcode.Config.Constants.Turret.*;
import static org.firstinspires.ftc.teamcode.Config.Utils.TurretMath.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;


public class Turret extends WSubsystem {
    private CRServo servo;
    private CRServo servo2;
    private AnalogInput servoEncoder;
    private PIDFController angleController;
    private Follower follower;

    // Configurable parameters
    private double targetAngle = 0; // field-relative angle the turret should face (wrapped -180 to 180)

    // Read variables (sensor values)
    private double heading = 0;
    private double rawServoAngle = 0; // wrapped 0-360 from analog encoder
    private double previousRawServoAngle = 0; // for wrap detection
    private double unwrappedServoAngle = 0; // accumulated total rotation (can be any value, like motor encoder)

    // Loop variables
    private double targetServoAngle = 0; // unwrapped target servo angle (can be any value)
    private double servoError = 0;
    private double currentTurretAngle = 0; // for telemetry

    // Write variables (actuator commands)
    private double servoPower = 0;



  public Turret(HardwareMap hardwareMap, Follower follower) {
      servo = hardwareMap.get(CRServo.class, servoName);
      servo2 = hardwareMap.get(CRServo.class, servoName2);
      servoEncoder = hardwareMap.get(AnalogInput.class, servoEncoderName);
      this.follower = follower;
      angleController = new PIDFController(kP, kI, kD, 0);

      // Initialize unwrapped angle tracking
      rawServoAngle = servoEncoder.getVoltage() / 3.3 * 360.0;
      previousRawServoAngle = rawServoAngle;
      unwrappedServoAngle = rawServoAngle; // Start at current position
      targetServoAngle = unwrappedServoAngle;
  }

  @Override
  public void read() {
    // Read robot heading from follower
    heading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

    // Read raw servo angle from absolute encoder (wrapped 0-360)
    rawServoAngle = servoEncoder.getVoltage() / 3.3 * 360.0;

    // Detect wraps and accumulate to create unwrapped angle (like motor encoder)
    double delta = rawServoAngle - previousRawServoAngle;

    // If we wrapped around (jumped from ~360 to ~0 or ~0 to ~360)
    if (delta > 180) {
      // Wrapped backwards (360 -> 0), we actually moved negative
      delta -= 360;
    } else if (delta < -180) {
      // Wrapped forwards (0 -> 360), we actually moved positive
      delta += 360;
    }

    // Update unwrapped angle by adding the actual movement
    unwrappedServoAngle += delta;

    // Store for next iteration
    previousRawServoAngle = rawServoAngle;
  }

  @Override
  public void loop() {
    // Calculate desired turret angle (robot-relative) to point to targetAngle (field-relative)
    double desiredTurretAngleRaw = targetAngle - heading;
    double wrappedDesiredTurretAngle = wrap180(desiredTurretAngleRaw);

    // Convert current servo angle to equivalent turret angle for telemetry
    currentTurretAngle = unwrappedServoAngle / gearRatio;
    double wrappedCurrentTurretAngle = wrap180(currentTurretAngle);

    // Smart range limiting: prevent wire damage by choosing safe boundary
    double safeTurretAngle = getSafeTurretAngle(wrappedDesiredTurretAngle, wrappedCurrentTurretAngle);

    // Find the closest unwrapped servo target that achieves the safe turret angle
    // This is the key: we work in unwrapped space like the motor encoder example
    double baseServoAngle = safeTurretAngle * gearRatio;

    // Find which 360° wrap we're currently in
    int currentWrapCount = (int) Math.round(unwrappedServoAngle / 360.0);

    // Check 3 candidates: one wrap behind, current wrap, one wrap ahead
    double[] candidates = {
      baseServoAngle + ((currentWrapCount - 1) * 360.0),
      baseServoAngle + (currentWrapCount * 360.0),
      baseServoAngle + ((currentWrapCount + 1) * 360.0)
    };

    // Choose the candidate with shortest distance that doesn't cross forbidden zone
    targetServoAngle = candidates[1]; // default to middle
    double minDistance = Double.MAX_VALUE;

    for (double candidate : candidates) {
      // Verify this candidate produces the correct turret angle
      double candidateTurretAngle = wrap180(candidate / gearRatio);
      if (Math.abs(candidateTurretAngle - safeTurretAngle) > 0.1) {
        continue; // Wrong angle due to wrapping
      }

      // Check if path is safe
      if (isPathSafe(unwrappedServoAngle, candidate)) {
        double distance = Math.abs(candidate - unwrappedServoAngle);
        if (distance < minDistance) {
          minDistance = distance;
          targetServoAngle = candidate;
        }
      }
    }

    // Calculate error directly in unwrapped space (like motor encoder code)
    servoError = targetServoAngle - unwrappedServoAngle;

    // Use PID controller with unwrapped values
    angleController.setSetPoint(targetServoAngle);
    servoPower = angleController.calculate(unwrappedServoAngle);

    // Add direction-specific feedforward term to overcome friction/deadband
    if (servoError > 0) {
      servoPower += kF_left;  // Moving left (positive power)
    } else if (servoError < 0) {
      servoPower += kF_right; // Moving right (negative power)
    }

    // Clamp servo power to valid range for CRServo
    servoPower = clamp(servoPower, -1.0, 1.0);
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
  private double getSafeTurretAngle(double desiredAngle, double currentAngle) {
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
   * 1. The target turret angle (wrapped) is within -135° to +135°
   * 2. The path doesn't cross through the forbidden zone at ANY point
   * 3. We check EVERY degree along the path to ensure no forbidden zone crossing
   *
   * @param currentServo Current unwrapped servo angle
   * @param targetServo Target unwrapped servo angle
   * @return true if the path is safe and target is valid
   */
  private boolean isPathSafe(double currentServo, double targetServo) {
    final double MIN_TURRET = -135.0;
    final double MAX_TURRET = 135.0;

    // Convert target to turret angle and check if it's valid
    double targetTurret = targetServo / gearRatio;
    double wrappedTargetTurret = wrap180(targetTurret);

    // First check: target must be within valid range
    if (wrappedTargetTurret < MIN_TURRET || wrappedTargetTurret > MAX_TURRET) {
      return false;
    }

    // Calculate the angular distance we'd travel (in servo degrees)
    double angularDistance = targetServo - currentServo;

    // Sample every 5 degrees of servo motion (turret moves slower due to gear ratio)
    // This ensures we catch any crossing of the forbidden zone
    int numSamples = Math.max(20, (int)(Math.abs(angularDistance) / 5.0));

    for (int i = 0; i <= numSamples; i++) {
      double t = i / (double) numSamples;
      double sampleServo = currentServo + t * angularDistance;
      double sampleTurret = sampleServo / gearRatio;
      double wrappedSample = wrap180(sampleTurret);

      // If any point along the path crosses into forbidden zone, path is unsafe
      if (wrappedSample < MIN_TURRET || wrappedSample > MAX_TURRET) {
        return false;
      }
    }

    return true;
  }

  @Override
  public void write() {
    // Set power to both servos
    servo.setPower(servoPower);
    servo2.setPower(servoPower);
  }

  // Public methods for controlling and querying turret state

  /**
   * Set the target angle (field-relative) for the turret to point to
   * @param angle Target angle in degrees (field-relative)
   */
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  /**
   * Get the current target angle (field-relative)
   * @return Target angle in degrees
   */
  public double getTargetAngle() {
    return targetAngle;
  }

  /**
   * Get the current turret angle (robot-relative)
   * @return Current turret angle in degrees
   */
  public double getCurrentTurretAngle() {
    return currentTurretAngle;
  }

  /**
   * Get the current servo power being applied
   * @return Servo power from -1.0 to 1.0
   */
  public double getServoPower() {
    return servoPower;
  }

  /**
   * Get the error between desired and current servo angle (unwrapped)
   * @return Error in degrees
   */
  public double getServoError() {
    return servoError;
  }

  public InstantCommand TargetAngle(double Angle){
      return new InstantCommand(()->setTargetAngle(Angle));
  }
}
