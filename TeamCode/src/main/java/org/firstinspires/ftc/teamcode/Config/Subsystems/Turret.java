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
    private double targetAngle = 0; // field-relative angle the turret should face

    // Read variables (sensor values)
    private double heading = 0;
    private double currentServoAngle = 0;

    private double desiredTurretAngle;
    private double desiredServoAngle;
    private double currentTurretAngle;
    private double wrappedServoError;
    private double adjustedTarget;

    // Write variables (actuator commands)
    private double servoPower = 0;



  public Turret(HardwareMap hardwareMap, Follower follower) {
      servo = hardwareMap.get(CRServo.class, servoName);
      servo2 = hardwareMap.get(CRServo.class, servoName2);
      servoEncoder = hardwareMap.get(AnalogInput.class, servoEncoderName);
      this.follower = follower;
      angleController = new PIDFController(kP, kI, kD, 0);
  }

  @Override
  public void read() {
    // Read robot heading from follower
    heading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

    // Read current servo angle from absolute encoder (in degrees)
    currentServoAngle = getUnwrappedServoAngle(servoEncoder);
  }

  @Override
  public void loop() {

    // Calculate desired turret angle (robot-relative) to point to targetAngle (field-relative)
    double desiredTurretAngleRaw = targetAngle - heading;
    double wrappedDesiredTurretAngle = wrap180(desiredTurretAngleRaw);

    // Convert current servo angle to equivalent turret angle for telemetry
    currentTurretAngle = currentServoAngle / gearRatio;

    // Wrap current turret angle to -180 to +180 for comparison
    double wrappedCurrentTurretAngle = wrap180(currentTurretAngle);

    // Smart range limiting: prevent wire damage by choosing safe boundary
    desiredTurretAngle = getSafeTurretAngle(wrappedDesiredTurretAngle, wrappedCurrentTurretAngle);

    // Find the unwrapped servo angle target that corresponds to the desired turret angle
    // We need to find which "wrap" of the desired angle is closest to our current position
    double baseServoAngle = desiredTurretAngle * gearRatio;

    // Find which 360° wrap the current servo position is in
    // This handles cases where servo has rotated many times (e.g., currentServoAngle = 1080°)
    int currentWrapCount = (int) Math.round(currentServoAngle / 360.0);

    // Generate candidates in both directions from current wrap
    // Check enough wraps to ensure we find all possible paths including unwrapping
    int searchRange = 3; // Check 3 wraps in each direction to handle edge cases

    java.util.List<Double> candidates = new java.util.ArrayList<>();
    for (int i = -searchRange; i <= searchRange; i++) {
      candidates.add(baseServoAngle + ((currentWrapCount + i) * 360.0));
    }

    // Find the valid candidate with the shortest safe path (supporting infinite wraps)
    desiredServoAngle = baseServoAngle + (currentWrapCount * 360.0); // default to middle candidate
    double minDistance = Double.MAX_VALUE;

    for (double candidate : candidates) {
      // First check: does this candidate produce the desired turret angle when wrapped?
      double candidateTurretAngle = wrap180(candidate / gearRatio);
      double tolerance = 0.1; // Allow small numerical errors

      // Skip candidates that don't match the desired turret angle
      if (Math.abs(candidateTurretAngle - desiredTurretAngle) > tolerance) {
        continue;
      }

      // Check if taking this path would cross the forbidden zone
      if (isPathSafe(currentServoAngle, candidate)) {
        double distance = Math.abs(candidate - currentServoAngle);
        if (distance < minDistance) {
          minDistance = distance;
          desiredServoAngle = candidate;
        }
      }
    }

    // Calculate servo error and set target
    wrappedServoError = desiredServoAngle - currentServoAngle;
    adjustedTarget = desiredServoAngle;

    // Use PID controller with current position and adjusted target
    angleController.setSetPoint(adjustedTarget);
    servoPower = angleController.calculate(currentServoAngle);

    // Add direction-specific feedforward term to overcome friction/deadband
    if (wrappedServoError > 0) {
      servoPower += kF_left;  // Moving left (positive power)
    } else if (wrappedServoError < 0) {
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
   * Get the desired turret angle after range limiting
   * @return Desired turret angle in degrees
   */
  public double getDesiredTurretAngle() {
    return desiredTurretAngle;
  }


  /**
   * Get the current servo power being applied
   * @return Servo power from -1.0 to 1.0
   */
  public double getServoPower() {
    return servoPower;
  }

  /**
   * Get the error between desired and current servo angle (wrapped)
   * @return Error in degrees
   */
  public double getServoError() {
    return wrappedServoError;
  }

  public InstantCommand TargetAngle(double Angle){
      return new InstantCommand(()->setTargetAngle(Angle));
  }
}
