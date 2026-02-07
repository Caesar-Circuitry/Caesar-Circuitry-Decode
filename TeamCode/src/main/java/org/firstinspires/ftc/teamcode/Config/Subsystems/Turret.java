package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.firstinspires.ftc.teamcode.Config.Constants.Turret.*;
import static org.firstinspires.ftc.teamcode.Config.Utils.TurretMath.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Config.Utils.AxonEncoder;
import org.firstinspires.ftc.teamcode.Config.Utils.AnglePIDF;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import java.util.LinkedList;


public class Turret extends WSubsystem {
  private CRServo servo;
  private CRServo servo2;
  private AxonEncoder turretEncoder;
  private AnglePIDF angleController;
  private Follower follower;
  private Launcher launcher; // For voltage compensation

  // Configuration
  private double targetAngle = 0;
  private double robotRelativeTargetAngle = 0;
  private boolean trackPinpoint = false;
  private Pose targetPose = null; // Target pose for continuous tracking

  // State
  private double heading = 0;
  private double currentTurretAngle = 0;
  private double targetServoAngle = 0;
  private double servoError = 0;
  private double servoPower = 0;
  private double voltageCompensation = 1.0;
  private boolean usingLargePID = true;

  // Constants
  private static final double SAFE_LIMIT = 135.0;
  private static final double ERROR_DEADBAND = 5.0;
  private static final double WAYPOINT_THRESHOLD = 20.0;
  private static final double NOMINAL_VOLTAGE = 12.82; // Nominal battery voltage for compensation

  // Wrap-around state
  private boolean isWrapping = false;
  private int wrapStep = 0; // 0=not wrapping, 1-4=waypoints
  private boolean wrapGoingPositive = false; // true = going toward +135, false = going toward -135

  // Waypoints: we always go through center
  // Going positive (from neg to pos): -135 → -10 → +10 → +135
  // Going negative (from pos to neg): +135 → +10 → -10 → -135

  // Telemetry
  private LinkedList<TelemetryPacket> telemetryPackets;
  private double unwrappedServoAngle = 0;
  private double desiredTurretAngle = 0;
  private double safeTurretAngle = 0;
  private double wrappedCurrentTurret = 0;


  public Turret(HardwareMap hardwareMap, Follower follower, Launcher launcher) {
    servo = hardwareMap.get(CRServo.class, servoName);
    servo2 = hardwareMap.get(CRServo.class, servoName2);
    turretEncoder = new AxonEncoder(hardwareMap.get(com.qualcomm.robotcore.hardware.AnalogInput.class, servoEncoderName), gearRatio, angleOffset);
    this.follower = follower;
    this.launcher = launcher; // Store launcher reference for voltage reading
    angleController = new AnglePIDF(kP, kI, kD, kF_left, kF_right);
    angleController.enableUnwrappedMode();
    telemetryPackets = new LinkedList<>();
    turretEncoder.update();
    targetServoAngle = turretEncoder.getUnwrappedEncoderAngle();
  }

  @Override
  public void read() {
    heading = Math.toDegrees(follower.poseTracker.getPose().getHeading());
    turretEncoder.update();
  }

  @Override
  public void loop() {
    // Get current position
    unwrappedServoAngle = turretEncoder.getUnwrappedEncoderAngle();
    currentTurretAngle = unwrappedServoAngle / gearRatio;
    wrappedCurrentTurret = wrap180(currentTurretAngle);

    // Get desired turret angle
    if (targetPose != null) {
      // Calculate field angle to target pose dynamically
      Pose robotPose = follower.poseTracker.getPose();
      double dx = targetPose.getX() - robotPose.getX();
      double dy = targetPose.getY() - robotPose.getY();
      double fieldAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
      // Negate the field angle, then convert to robot-relative with 180° offset (turret 0° faces robot's back)
      desiredTurretAngle = wrap180(-fieldAngleToTarget - heading + 180.0);
    } else if (trackPinpoint) {
      // Add 180° offset for turret mounting (0° = robot's back)
      desiredTurretAngle = wrap180(targetAngle - heading + 180.0);
    } else {
      desiredTurretAngle = robotRelativeTargetAngle;
    }

    // Determine safe turret angle
    safeTurretAngle = calculateSafeTurretAngle(wrappedCurrentTurret, desiredTurretAngle);

    // Handle wrap state machine if wrapping
    if (isWrapping) {
      double waypoint = getCurrentWaypoint();

      // Check if we reached the waypoint
      if (Math.abs(wrappedCurrentTurret - waypoint) < WAYPOINT_THRESHOLD) {
        wrapStep++;
        if (wrapStep > 4) {
          // Done wrapping
          isWrapping = false;
          wrapStep = 0;
        }
      }

      // Override safe target with current waypoint while wrapping
      if (isWrapping) {
        safeTurretAngle = getCurrentWaypoint();
      }
    }

    // Calculate target servo angle
    targetServoAngle = calculateServoTarget(safeTurretAngle);

    // Calculate error and power
    servoError = targetServoAngle - unwrappedServoAngle;

    // Apply voltage compensation to feedforward values
    voltageCompensation = 1.0;
    if (launcher != null) {
      double batteryVoltage = launcher.getBatteryVoltageValue();
      if (batteryVoltage > 0.1) {
        voltageCompensation = NOMINAL_VOLTAGE / batteryVoltage;
      }
    }

    // Select large or small PID based on error magnitude
    // Large PID for fast response when far from target
    // Small PID for fine tuning when close to target
    double currentKp, currentKi, currentKd;
    usingLargePID = Math.abs(servoError) > ERROR_THRESHOLD;

    if (usingLargePID) {
      currentKp = kP_large;
      currentKi = kI_large;
      currentKd = kD_large;
    } else {
      currentKp = kP_small;
      currentKi = kI_small;
      currentKd = kD_small;
    }

    // Update controller with selected PID and voltage-compensated kF values
    double compensatedKfLeft = kF_left * voltageCompensation;
    double compensatedKfRight = kF_right * voltageCompensation;
    angleController.setCoefficients(currentKp, currentKi, currentKd, compensatedKfLeft, compensatedKfRight);

    // PID control
    angleController.setSetPoint(targetServoAngle);
    double basePower;

    if (Math.abs(servoError) < ERROR_DEADBAND) {
      basePower = 0.0;
      angleController.reset();
    } else {
      basePower = angleController.calculate(unwrappedServoAngle);
      // Limit power when using small PID (close to target) to reduce overshoot
      if (!usingLargePID) {
        basePower = clamp(basePower, -0.5, 0.5);
      }
    }

    // SAFETY: If in forbidden zone (>140°), force movement toward center
    boolean inForbiddenZone = Math.abs(wrappedCurrentTurret) > 140.0;
    if (inForbiddenZone) {
      if (wrappedCurrentTurret > 0) {
        basePower = -0.5;
      } else {
        basePower = 0.5;
      }
    }

    servoPower = clamp(basePower, -1.0, 1.0);

    // Telemetry
    if (logTelemetry) {
      telemetryPackets.clear();

      String status;
      if (inForbiddenZone) {
        status = "FORBIDDEN ZONE";
      } else if (isWrapping) {
        status = "WRAPPING Step " + wrapStep + " → " + getCurrentWaypoint() + "°";
      } else if (Math.abs(servoError) < ERROR_DEADBAND) {
        status = "AT TARGET";
      } else {
        status = "MOVING";
      }

      telemetryPackets.addLast(new TelemetryPacket("=== STATUS ===", status));
      telemetryPackets.addLast(new TelemetryPacket("Wrapped Turret", wrappedCurrentTurret));
      telemetryPackets.addLast(new TelemetryPacket("Desired Turret", desiredTurretAngle));
      telemetryPackets.addLast(new TelemetryPacket("Safe Turret", safeTurretAngle));
      telemetryPackets.addLast(new TelemetryPacket("Servo Error", servoError));
      telemetryPackets.addLast(new TelemetryPacket("Servo Power", servoPower));
      telemetryPackets.addLast(new TelemetryPacket("Is Wrapping", isWrapping));
      telemetryPackets.addLast(new TelemetryPacket("Wrap Step", wrapStep));
      telemetryPackets.addLast(new TelemetryPacket("Wrap Direction", wrapGoingPositive ? "→ +135" : "→ -135"));
      telemetryPackets.addLast(new TelemetryPacket("Unwrapped Servo", unwrappedServoAngle));
      telemetryPackets.addLast(new TelemetryPacket("Target Servo", targetServoAngle));
      telemetryPackets.addLast(new TelemetryPacket("In Forbidden Zone", inForbiddenZone));
      telemetryPackets.addLast(new TelemetryPacket("PID Mode", usingLargePID ? "LARGE" : "SMALL"));
      telemetryPackets.addLast(new TelemetryPacket("Battery Voltage", launcher != null ? launcher.getBatteryVoltageValue() : NOMINAL_VOLTAGE));
      telemetryPackets.addLast(new TelemetryPacket("Voltage Compensation", voltageCompensation));
      telemetryPackets.addLast(new TelemetryPacket("Heading", heading));
    }
  }

  /**
   * Calculate the safe turret angle, handling wrap-around logic
   *
   * RULES:
   * 1. If target is in forbidden zone (>±135°), go to ±135° on the SAME SIDE AS TARGET
   * 2. If target is reachable without crossing forbidden zone, go direct
   * 3. If target requires crossing through ±180° (forbidden), start wrap sequence
   */
  private double calculateSafeTurretAngle(double current, double desired) {
    // First, check if desired is in forbidden zone
    boolean desiredInForbidden = Math.abs(desired) > SAFE_LIMIT;

    if (desiredInForbidden) {
      // Target is in forbidden zone - go to the limit on the SAME SIDE AS TARGET
      // This way we get as close as possible to where we want to be
      if (desired > 0) {
        // Target is past +135°, go to +135°
        // But we might need to wrap if we're on the negative side
        if (current < -90.0) {
          // We're far on negative side, need to wrap to reach +135
          if (!isWrapping) {
            isWrapping = true;
            wrapStep = 1;
            wrapGoingPositive = true;
            return getCurrentWaypoint();
          }
        }
        return SAFE_LIMIT;  // Go to +135
      } else {
        // Target is past -135°, go to -135°
        // But we might need to wrap if we're on the positive side
        if (current > 90.0) {
          // We're far on positive side, need to wrap to reach -135
          if (!isWrapping) {
            isWrapping = true;
            wrapStep = 1;
            wrapGoingPositive = false;
            return getCurrentWaypoint();
          }
        }
        return -SAFE_LIMIT; // Go to -135
      }
    }

    // Desired is within safe range (-135 to +135)
    // Check if we can reach it directly without crossing forbidden zone

    boolean currentPositive = current >= 0;
    boolean desiredPositive = desired >= 0;

    // If same side, always safe to go direct
    if (currentPositive == desiredPositive) {
      if (!isWrapping) {
        return desired;
      }
    }

    // Opposite sides - check if we need to wrap
    // We need to wrap if BOTH are far from center (>90°)
    // because the direct path would cross through ±180° (forbidden)
    boolean needsWrap = Math.abs(current) > 90.0 && Math.abs(desired) > 90.0;

    if (needsWrap && !isWrapping) {
      // Start wrap sequence
      isWrapping = true;
      wrapStep = 1;
      wrapGoingPositive = desired > 0; // Going toward the target side
      return getCurrentWaypoint();
    }

    if (!isWrapping) {
      // Safe to go direct (one of us is near center)
      return desired;
    }

    // If we're wrapping, the waypoint is returned in loop()
    return getCurrentWaypoint();
  }

  /**
   * Get the current waypoint for wrap sequence
   */
  private double getCurrentWaypoint() {
    if (!isWrapping || wrapStep < 1 || wrapStep > 4) {
      return safeTurretAngle;
    }

    if (wrapGoingPositive) {
      // Going from negative to positive: -135 → -10 → +10 → +135
      switch (wrapStep) {
        case 1: return -SAFE_LIMIT; // -135
        case 2: return -10.0;
        case 3: return 10.0;
        case 4: return SAFE_LIMIT;  // +135
        default: return 0;
      }
    } else {
      // Going from positive to negative: +135 → +10 → -10 → -135
      switch (wrapStep) {
        case 1: return SAFE_LIMIT;  // +135
        case 2: return 10.0;
        case 3: return -10.0;
        case 4: return -SAFE_LIMIT; // -135
        default: return 0;
      }
    }
  }

  /**
   * Calculate the servo target for a given turret angle
   */
  private double calculateServoTarget(double turretTarget) {
    double targetTurretServo = turretTarget * gearRatio;

    // Find which 720° wrap we're in
    double servoPerTurretRotation = 360.0 * gearRatio;
    int turretWrap = (int) Math.round(unwrappedServoAngle / servoPerTurretRotation);

    // Calculate candidates
    double candidate1 = targetTurretServo + (turretWrap * servoPerTurretRotation);
    double candidate2 = targetTurretServo + ((turretWrap - 1) * servoPerTurretRotation);
    double candidate3 = targetTurretServo + ((turretWrap + 1) * servoPerTurretRotation);

    // Pick closest
    double dist1 = Math.abs(candidate1 - unwrappedServoAngle);
    double dist2 = Math.abs(candidate2 - unwrappedServoAngle);
    double dist3 = Math.abs(candidate3 - unwrappedServoAngle);

    if (dist1 <= dist2 && dist1 <= dist3) {
      return candidate1;
    } else if (dist2 <= dist3) {
      return candidate2;
    } else {
      return candidate3;
    }
  }

  @Override
  public void write() {
    servo.setPower(servoPower);
    servo2.setPower(servoPower);
  }

  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
    this.robotRelativeTargetAngle = wrap180(angle - heading);
    this.trackPinpoint = false;
    this.targetPose = null; // Clear target pose tracking
    this.isWrapping = false;
    this.wrapStep = 0;
  }

  public void faceTarget(Pose targetPose, Pose robotPose) {
    // Store the target pose for continuous tracking
    // The loop() will calculate the angle to this pose every cycle
    this.targetPose = targetPose;
    this.trackPinpoint = false; // Not using fixed field angle
    this.isWrapping = false;
    this.wrapStep = 0;
  }

  /**
   * Set a target pose to continuously track
   * The turret will automatically calculate the angle to this pose as the robot moves
   * @param targetPose The pose to point at
   */
  public void setTargetPose(Pose targetPose) {
    this.targetPose = targetPose;
    this.trackPinpoint = false;
    this.isWrapping = false;
    this.wrapStep = 0;
  }

  /**
   * Clear the target pose and stop tracking
   */
  public void clearTargetPose() {
    this.targetPose = null;
  }

  public void enablePinpointTracking() {
    this.trackPinpoint = true;
  }

  public void disablePinpointTracking() {
    this.trackPinpoint = false;
  }

  public boolean isPinpointTrackingEnabled() {
    return trackPinpoint;
  }

  @Override
  public LinkedList<TelemetryPacket> getTelemetry() {
    return telemetryPackets;
  }

  public InstantCommand TargetAngle(double Angle) {
    return new InstantCommand(() -> setTargetAngle(Angle));
  }

  public double getCurrentTurretAngle() {
    return currentTurretAngle;
  }
}
