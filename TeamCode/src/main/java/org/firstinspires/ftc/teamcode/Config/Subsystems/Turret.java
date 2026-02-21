package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.firstinspires.ftc.teamcode.Config.Constants.Turret.*;
import static org.firstinspires.ftc.teamcode.Config.Utils.TurretMath.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Config.Utils.ThroughBoreEncoder;
import org.firstinspires.ftc.teamcode.Config.Utils.AnglePIDF;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import java.util.LinkedList;


public class Turret extends WSubsystem {
  private CRServo servo;
  private CRServo servo2;
  private ThroughBoreEncoder turretEncoder;
  private AnglePIDF angleController;
  private Follower follower;
  private Launcher launcher; // For voltage compensation

  // Configuration
  private double targetAngle = 0;
  private double robotRelativeTargetAngle = 0;
  private boolean trackPinpoint = true;
  private Pose targetPose = null; // Target pose for continuous tracking

  // State
  private double heading = 0;
  private double filteredHeading = 0; // Low-pass filtered heading
  private static final double HEADING_FILTER_ALPHA = 0.3; // 0 = no filtering, 1 = no smoothing
  private double currentTurretAngle = 0;
  private double targetServoAngle = 0;
  private double servoError = 0;
  private double servoPower = 0;
  private double voltageCompensation = 1.0;
  private boolean usingLargePID = true;

  // Constants
  private static final double SAFE_LIMIT = 135.0;
  private static final double DEADBAND = 4.0; // ~2° turret error - stop completely
  private static final double WAYPOINT_THRESHOLD = 35.0; // In turret space - for waypoint detection
  private static final double NOMINAL_VOLTAGE = 12.3; // Nominal battery voltage for compensation

  // Wrap-around state
  private boolean isWrapping = false;
  private int wrapStep = 0; // 0=not wrapping, 1-3=waypoints
  private boolean wrapGoingPositive = false; // true = going toward +135, false = going toward -135
  private boolean justFinishedWrap = false; // Prevents immediate re-wrap after completing

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
    DcMotorEx encoderMotor = hardwareMap.get(DcMotorEx.class, encoderMotorName);
    turretEncoder = new ThroughBoreEncoder(encoderMotor, gearRatio, TICKS_PER_REV, true);
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
    double rawHeading = Math.toDegrees(follower.poseTracker.getPose().getHeading());
    // Apply low-pass filter to reduce noise-induced oscillation
    // Handle angle wrapping for the filter
    double headingDiff = wrap180(rawHeading - filteredHeading);
    filteredHeading = wrap180(filteredHeading + HEADING_FILTER_ALPHA * headingDiff);
    heading = filteredHeading;
    turretEncoder.update();
  }

  @Override
  public void loop() {
    // Get current position
    unwrappedServoAngle = turretEncoder.getUnwrappedEncoderAngle();
    currentTurretAngle = unwrappedServoAngle / gearRatio;
    wrappedCurrentTurret = wrap180(currentTurretAngle);

    // Get desired turret angle
    boolean usingHeadingCompensation = false;
    if (targetPose != null) {
      // Calculate field angle to target pose dynamically
      Pose robotPose = follower.poseTracker.getPose();
      double dx = targetPose.getX() - robotPose.getX();
      double dy = targetPose.getY() - robotPose.getY();
      double fieldAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
      // Convert field angle to robot-relative: subtract robot heading
      desiredTurretAngle = wrap180(fieldAngleToTarget - heading);
      usingHeadingCompensation = true;
    } else if (trackPinpoint) {
      // Convert target field angle to robot-relative
      desiredTurretAngle = wrap180(targetAngle - heading);
      usingHeadingCompensation = true;
    } else {
      desiredTurretAngle = robotRelativeTargetAngle;
    }

    // When using heading compensation, be more careful about starting wraps
    // But if we're already wrapping, let it complete
    if (usingHeadingCompensation && !isWrapping) {
      // Clear cooldown if we're now on the same side as target or near center
      if (justFinishedWrap) {
        boolean sameSide = (wrappedCurrentTurret > 0) == (desiredTurretAngle > 0);
        boolean nearCenter = Math.abs(wrappedCurrentTurret) < 60.0;
        if (sameSide || nearCenter) {
          justFinishedWrap = false;
        }
      }

      // Not currently wrapping - check if we NEED to wrap
      // Don't start a new wrap if we just finished one (cooldown)
      boolean needsWrap = !justFinishedWrap &&
                          Math.abs(wrappedCurrentTurret) > 90.0 &&
                          Math.abs(desiredTurretAngle) > 90.0 &&
                          (wrappedCurrentTurret > 0) != (desiredTurretAngle > 0);

      if (needsWrap) {
        // Start wrapping even in heading compensation mode
        isWrapping = true;
        wrapStep = 1;
        wrapGoingPositive = desiredTurretAngle > 0;
        safeTurretAngle = getCurrentWaypoint();
      } else {
        // Clamp to safe limits
        if (desiredTurretAngle > SAFE_LIMIT) {
          safeTurretAngle = SAFE_LIMIT;
        } else if (desiredTurretAngle < -SAFE_LIMIT) {
          safeTurretAngle = -SAFE_LIMIT;
        } else {
          safeTurretAngle = desiredTurretAngle;
        }
      }
    } else if (isWrapping) {
      // Already wrapping - continue with wrap logic (handled below)
      safeTurretAngle = getCurrentWaypoint();
    } else {
      // For robot-relative commands, use full wrap logic
      safeTurretAngle = calculateSafeTurretAngle(wrappedCurrentTurret, desiredTurretAngle);
    }

    // Handle wrap state machine if wrapping
    if (isWrapping) {
      double waypoint = getCurrentWaypoint(); // Always 0 (center)
      safeTurretAngle = waypoint;

      // Check if we reached center (within threshold)
      if (Math.abs(wrappedCurrentTurret) < WAYPOINT_THRESHOLD) {
        // We're at center - done wrapping, now go to target
        isWrapping = false;
        wrapStep = 0;
        justFinishedWrap = true; // Set cooldown to prevent immediate re-wrap

        // Clamp and go to desired
        if (desiredTurretAngle > SAFE_LIMIT) {
          safeTurretAngle = SAFE_LIMIT;
        } else if (desiredTurretAngle < -SAFE_LIMIT) {
          safeTurretAngle = -SAFE_LIMIT;
        } else {
          safeTurretAngle = desiredTurretAngle;
        }
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

    // Select large or small PID based on error magnitude with hysteresis
    // Hysteresis prevents rapid switching: switch to SMALL at 25°, switch to LARGE at 40°
    double currentKp, currentKi, currentKd;
    double absServoError = Math.abs(servoError);

    if (usingLargePID) {
      // Currently using large PID - switch to small when error drops below 25°
      if (absServoError < 25.0) {
        usingLargePID = false;
      }
    } else {
      // Currently using small PID - switch to large when error exceeds 40°
      if (absServoError > 40.0) {
        usingLargePID = true;
      }
    }

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

    // PID control - large or small PID with deadband
    angleController.setSetPoint(targetServoAngle);
    double basePower;
    double absError = Math.abs(servoError);

    if (absError < DEADBAND) {
      basePower = 0.0;
      angleController.reset();
    } else {
      basePower = angleController.calculate(unwrappedServoAngle);
      if (usingLargePID) {
        basePower = clamp(basePower, -1.0, 1.0);
      } else {
        basePower = clamp(basePower, -0.6, 0.6);
      }
    }

    // SAFETY: Forbidden zone correction when past ±130°
    boolean inForbiddenZone = false;
    double absCurrentAngle = Math.abs(wrappedCurrentTurret);

    if (absCurrentAngle > 130.0) {
      inForbiddenZone = true;
      double overrunFactor = clamp((absCurrentAngle - 130.0) / 20.0, 0.0, 1.0);
      double correctivePower = 0.15 + (overrunFactor * 0.2);

      if (wrappedCurrentTurret > 0) {
        basePower = Math.min(basePower, -correctivePower);
      } else {
        basePower = Math.max(basePower, correctivePower);
      }
    }

    servoPower = clamp(basePower, -1.0, 1.0);

    // Telemetry
    if (logTelemetry) {
      telemetryPackets.clear();

      String status;
      if (absCurrentAngle > 145.0) {
        status = "FORBIDDEN ZONE (CRITICAL)";
      } else if (inForbiddenZone) {
        status = "FORBIDDEN ZONE - CORRECTING";
      } else if (isWrapping) {
        status = "WRAPPING";
      } else if (absError < DEADBAND) {
        status = "AT TARGET";
      } else {
        status = usingLargePID ? "MOVING (LARGE PID)" : "MOVING (SMALL PID)";
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
   * 1. If we can go direct (not crossing forbidden zone), do it
   * 2. If we need to cross through ±180° (forbidden), go through center first
   * 3. Always clamp final result to ±SAFE_LIMIT
   */
  private double calculateSafeTurretAngle(double current, double desired) {
    // Clamp desired to safe limits first
    double clampedDesired = desired;
    if (desired > SAFE_LIMIT) {
      clampedDesired = SAFE_LIMIT;
    } else if (desired < -SAFE_LIMIT) {
      clampedDesired = -SAFE_LIMIT;
    }

    // If already wrapping, continue with waypoints
    if (isWrapping) {
      return getCurrentWaypoint();
    }

    // Check if we're currently in the danger zone (close to ±180)
    // If so, move toward safe limit first
    if (Math.abs(current) > SAFE_LIMIT) {
      return current > 0 ? SAFE_LIMIT : -SAFE_LIMIT;
    }

    // If either is near center (within ±90°), we can go direct
    if (Math.abs(current) <= 90.0 || Math.abs(clampedDesired) <= 90.0) {
      return clampedDesired;
    }

    // Both are far from center (>90°)
    boolean currentPositive = current > 0;
    boolean desiredPositive = clampedDesired > 0;

    // If same side, go direct
    if (currentPositive == desiredPositive) {
      return clampedDesired;
    }

    // Opposite sides AND both far from center - need to wrap through center
    // Start wrapping sequence
    isWrapping = true;
    wrapStep = 1;
    wrapGoingPositive = desiredPositive;

    return getCurrentWaypoint();
  }




  /**
   * Get the current waypoint for wrap sequence
   * Simplified 2-step: just go to center (0°), then to target
   */
  private double getCurrentWaypoint() {
    if (!isWrapping || wrapStep < 1 || wrapStep > 2) {
      return 0.0;
    }

    // Step 1: Go to center
    // Step 2: Done - handled by wrap completion logic
    return 0.0; // Always go to center first
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

  public InstantCommand TargetBlueGoal() {
    return new InstantCommand(() -> setTargetPose(org.firstinspires.ftc.teamcode.Config.Constants.Robot.BlueGoal));
  }

  public InstantCommand TargetRedGoal() {
    return new InstantCommand(() -> setTargetPose(org.firstinspires.ftc.teamcode.Config.Constants.Robot.RedGoal));
  }

  public InstantCommand StopTracking() {
    return new InstantCommand(() -> clearTargetPose());
  }

  public double getCurrentTurretAngle() {
    return currentTurretAngle;
  }
}
