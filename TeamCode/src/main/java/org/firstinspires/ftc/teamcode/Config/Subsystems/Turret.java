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

      // Initialize unwrapped angle tracking - read raw 0-360 value first
      rawServoAngle = servoEncoder.getVoltage() / 3.3 * 360.0;
      previousRawServoAngle = rawServoAngle;
      // Apply 180-degree offset to the unwrapped angle (for 90-degree turret offset via 2:1 ratio)
      unwrappedServoAngle = rawServoAngle + 180.0;
      targetServoAngle = unwrappedServoAngle;
  }

  @Override
  public void read() {
    // Read robot heading from follower
    heading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

    // Read raw servo angle from absolute encoder (0-360, NO offset yet)
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
    // The unwrapped angle maintains the 180° offset that was set in the constructor
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
    // The 180-degree offset is already applied to unwrappedServoAngle, just divide by gear ratio
    currentTurretAngle = unwrappedServoAngle / gearRatio;
    double wrappedCurrentTurretAngle = wrap180(currentTurretAngle);

    // Smart range limiting: prevent wire damage by choosing safe boundary
    double safeTurretAngle = getSafeTurretAngle(wrappedDesiredTurretAngle, wrappedCurrentTurretAngle);

    // Convert desired turret angle to servo angle
    // Just multiply by gear ratio - the offset is already in the servo coordinate system
    double baseServoAngle = safeTurretAngle * gearRatio;

    // Find which 360° wrap we're currently in
    int currentWrapCount = (int) Math.round(unwrappedServoAngle / 360.0);

    // Check 3 candidates: one wrap behind, current wrap, one wrap ahead
    double[] candidates = {
      baseServoAngle + ((currentWrapCount - 1) * 360.0),
      baseServoAngle + (currentWrapCount * 360.0),
      baseServoAngle + ((currentWrapCount + 1) * 360.0)
    };

    // Choose the candidate with LONGEST distance that is safe (opposite of shortest path)
    targetServoAngle = candidates[1]; // default to middle
    double maxDistance = -1.0;

    for (double candidate : candidates) {
      // Verify this candidate produces the correct turret angle
      // The offset is already in the candidate value, just divide by gear ratio
      double candidateTurretAngle = wrap180(candidate / gearRatio);
      if (Math.abs(candidateTurretAngle - safeTurretAngle) > 0.1) {
        continue; // Wrong angle due to wrapping
      }

      // Check if path is safe
      if (isPathSafe(unwrappedServoAngle, candidate, gearRatio)) {
        double distance = Math.abs(candidate - unwrappedServoAngle);
        if (distance > maxDistance) {
          maxDistance = distance;
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
