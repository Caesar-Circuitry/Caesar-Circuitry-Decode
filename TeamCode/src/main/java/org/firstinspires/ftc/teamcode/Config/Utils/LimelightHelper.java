package org.firstinspires.ftc.teamcode.Config.Utils;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;



public class LimelightHelper {

  // ==========================
  // VARIABLES
  // ==========================
  private final Limelight3A limelight;
  // replace IMUHelper with a PedroLocalizer abstraction
  private final PedroLocalizer pedroLocalizer;
  // use PIDController objects directly (injected)
  private PIDController distancePID;
  private PIDController yawPID;

  private double lastDistanceError = 0, distanceIntegral = 0;
  private double lastAngleError = 0, angleIntegral = 0;

  private DcMotor leftMotor;
  private DcMotor rightMotor;

  // ==========================
  // CONSTRUCTOR
  // ==========================

  /**
   * New constructor: accept a PedroLocalizer and optional PIDController instances.
   * If a PIDController is not provided, movement methods will use simple fallbacks.
   */
  public LimelightHelper(HardwareMap hardwareMap, String cameraName,
                         PedroLocalizer pedroLocalizer,
                         PIDController distancePID, PIDController yawPID) {
    this.limelight = hardwareMap.get(Limelight3A.class, cameraName);
    this.pedroLocalizer = pedroLocalizer;
    this.distancePID = distancePID;
    this.yawPID = yawPID;
  }

  // ==========================
  // LIMELIGHT DATA ACCESS
  // ==========================

  /** Returns the horizontal offset from the target (tx) */
  public double getTx() {
    return limelight.getLatestResult().getTx();
  }

  /** Returns the vertical offset from the target (ty) */
  public double getTy() {
    return limelight.getLatestResult().getTy();
  }

  /** Returns the detected target area (ta) */
  public double getTa() {
    return limelight.getLatestResult().getTa();
  }

  /** Checks if there is a valid target */
  public boolean isTargetValid() {
    return limelight.getLatestResult().isValid();
  }

  /** Returns the robot’s 3D pose (MT1) */
  public Pose3D getBotPoseMT1() {
    LLResult r = limelight.getLatestResult();
    if (r != null && r.isValid()) return r.getBotpose();
    return null;
  }

  /** Returns the robot’s 3D pose (MT2) using IMU orientation */
  public Pose3D getBotPoseMT2() {
    updateRobotOrientation();
    LLResult r = limelight.getLatestResult();
    if (r != null && r.isValid()) return r.getBotpose_MT2();
    return null;
  }

  /** Returns a list of detected fiducials (AprilTags) */
  public List<LLResultTypes.FiducialResult> getFiducials() {
    LLResult result = limelight.getLatestResult();
    if (result != null && result.isValid()) return result.getFiducialResults();
    return null;
  }

  /** Returns the distance to a given fiducial */
  public double getDistanceToFiducial(LLResultTypes.FiducialResult fiducial) {
    if (fiducial != null) {
      Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
      return Math.sqrt(
              robotPose.getPosition().x * robotPose.getPosition().x +
                      robotPose.getPosition().y * robotPose.getPosition().y +
                      robotPose.getPosition().z * robotPose.getPosition().z
      );
    }
    return -1;
  }

  /** Returns the yaw (horizontal angle) to the fiducial */
  public double getYawToFiducial(LLResultTypes.FiducialResult fiducial) {
    if (fiducial != null) {
      Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
      return Math.toDegrees(Math.atan2(robotPose.getPosition().y, robotPose.getPosition().x));
    }
    return 0;
  }

  /** Returns the pitch (vertical angle) to the fiducial */
  public double getPitchToFiducial(LLResultTypes.FiducialResult fiducial) {
    if (fiducial != null) {
      Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
      double horizontalDist = Math.sqrt(
              robotPose.getPosition().x * robotPose.getPosition().x +
                      robotPose.getPosition().y * robotPose.getPosition().y
      );
      return Math.toDegrees(Math.atan2(robotPose.getPosition().z, horizontalDist));
    }
    return 0;
  }

  /** Returns the estimated robot position on the field */
  public Pose3D getEstimatedFieldPosition() {
    updateRobotOrientation();
    LLResult result = limelight.getLatestResult();
    if (result != null && result.isValid()) {
      Pose3D mt2 = result.getBotpose_MT2();
      if (mt2 != null) return mt2;
      Pose3D mt1 = result.getBotpose();
      if (mt1 != null) return mt1;
    }
    return null;
  }

  // ==========================
  // PEDRO LOCALIZER TYPES
  // ==========================

  /** Minimal Pedro-compatible 2D pose (heading in radians). */
  public static class PedroPose2d {
    public double x;
    public double y;
    public double heading; // radians
    public PedroPose2d(double x, double y, double heading) { this.x = x; this.y = y; this.heading = heading; }
  }

  /** Minimal Pedro localizer interface so any Pedro localization source can be injected. */
  public interface PedroLocalizer {
    PedroPose2d getPose();          // may return null
    boolean update();               // optional update call, return true if pose available
  }

  // ==========================
  // TELEMETRY
  // ==========================

  /** Sends full positional and fiducial telemetry */
  public void telemetryFull(Telemetry telemetry) {
    Pose3D fieldPose = getEstimatedFieldPosition();
    if (fieldPose != null) {
      telemetry.addData("Robot (Field)", "X: %.2f Y: %.2f Z: %.2f",
              fieldPose.getPosition().x,
              fieldPose.getPosition().y,
              fieldPose.getPosition().z);
    }

    List<LLResultTypes.FiducialResult> fiducials = getFiducials();
    if (fiducials != null) {
      for (LLResultTypes.FiducialResult f : fiducials) {
        double dist = getDistanceToFiducial(f);
        double yaw = getYawToFiducial(f);
        double pitch = getPitchToFiducial(f);
        telemetry.addData("Fiducial " + f.getFiducialId(),
                String.format("Dist: %.2fm Yaw: %.1f° Pitch: %.1f°", dist, yaw, pitch));
      }
    }
  }


  // ==========================
  // MOVEMENT CALCULATIONS
  // ==========================

  /**
   * Calculates the direction and speed required to move to a target field position.
   * @return array [vForward, vStrafe, vRot]
   */
  public double[] calculateMovementToPosition(double xTarget, double yTarget, double dtSeconds) {
    Pose3D currentPose = getEstimatedFieldPosition();
    if (currentPose == null) return new double[]{0, 0, 0};

    double dx = xTarget - currentPose.getPosition().x;
    double dy = yTarget - currentPose.getPosition().y;
    double distance = Math.sqrt(dx * dx + dy * dy);
    double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
    double robotYawDeg = 0.0;
    PedroPose2d p = (pedroLocalizer != null) ? pedroLocalizer.getPose() : null;
    if (p != null) robotYawDeg = Math.toDegrees(p.heading);
    double yawError = normalizeAngleDeg(angleToTargetDeg - robotYawDeg);

    // PID - distance
    distanceIntegral += distance * dtSeconds;
    double distanceDerivative = (distance - lastDistanceError) / dtSeconds;
    lastDistanceError = distance;
    double vForward;
    if (distancePID != null) {
      // many PIDController APIs accept (setpoint, measurement) — adjust calls as needed in your project
      vForward = distancePID.calculate(0.0, -distance); // fallback usage: negative distance toward setpoint 0
    } else {
      vForward = distance * 0.5; // simple proportional fallback
    }

    // PID - rotation
    angleIntegral += yawError * dtSeconds;
    double angleDerivative = (yawError - lastAngleError) / dtSeconds;
    lastAngleError = yawError;
    double vRot;
    if (yawPID != null) {
      vRot = yawPID.calculate(0.0, yawError);
    } else {
      vRot = yawError * 0.01;
    }

    // Clamp outputs
    vForward = Math.min(vForward, 1);
    vRot = Math.max(Math.min(vRot, 1), -1);

    return new double[]{vForward, 0, vRot}; // [forward, strafe (0 for tank), rotation]
  }

  // normalize to [-180,180]
  private double normalizeAngleDeg(double ang) {
    ang = ((ang + 180) % 360);
    if (ang < 0) ang += 360;
    return ang - 180;
  }

  // ==========================
  // PEDRO PATHING ADAPTERS
  // ==========================

  /** Returns a PedroPose2d using the injected PedroLocalizer when available, otherwise Limelight pose. */
  public PedroPose2d getPedroPose2d() {
    if (pedroLocalizer != null) return pedroLocalizer.getPose();
    Pose3D fieldPose = getEstimatedFieldPosition();
    if (fieldPose == null) return null;
    double x = fieldPose.getPosition().x;
    double y = fieldPose.getPosition().y;
    return new PedroPose2d(x, y, 0.0);
  }

  /**
   * Minimal localizer adapter that exposes the Limelight as a Pedro-friendly localizer.
   * The adapter is intentionally small: update() checks for new pose and getPose() returns the latest.
   */
  public class LimelightPedroLocalizer implements PedroLocalizer {
    private PedroPose2d lastPose = null;

    /** Polls the Limelight and updates internal pose. Returns true if a valid pose was obtained. */
    public boolean update() {
      PedroPose2d p = getPedroPose2d();
      if (p == null) return false;
      lastPose = p;
      return true;
    }

    /** Returns last known PedroPose2d or null. */
    public PedroPose2d getPose() {
      return lastPose;
    }

    /** Force-set the internal pose (useful to seed odometry). */
    public void setPose(double x, double y, double headingRadians) {
      lastPose = new PedroPose2d(x, y, headingRadians);
    }
  }

  /** Factory accessor for the LimelightPedroLocalizer */
  public LimelightPedroLocalizer getPedroLocalizer() {
    return new LimelightPedroLocalizer();
  }

  /**
   * Generates chassis velocities suitable for Pedro followers.
   * Input targets are in field coordinates (meters). Output is robot-frame velocities:
   * [vx (m/s forward), vy (m/s strafe to the right), omega (rad/s CCW)]
   *
   * This reuses the existing PID approach but rotates the forward/strafe into robot frame
   * using the IMU yaw.
   */
  public double[] getPedroDriveSignal(double xTarget, double yTarget, double dtSeconds) {
    Pose3D currentPose = getEstimatedFieldPosition();
    if (currentPose == null) return new double[]{0, 0, 0};

    double dx = xTarget - currentPose.getPosition().x;
    double dy = yTarget - currentPose.getPosition().y;
    double distance = Math.sqrt(dx * dx + dy * dy);
    double angleToTarget = Math.atan2(dy, dx); // radians
    double robotYawRad = 0.0;
    PedroPose2d p = (pedroLocalizer != null) ? pedroLocalizer.getPose() : null;
    if (p != null) robotYawRad = p.heading;
    double yawErrorDeg = normalizeAngleDeg(Math.toDegrees(angleToTarget) - Math.toDegrees(robotYawRad));

    // distance PID (reuse previous integrators)
    distanceIntegral += distance * dtSeconds;
    double distanceDerivative = (distance - lastDistanceError) / Math.max(dtSeconds, 1e-6);
    lastDistanceError = distance;
    double forwardSpeed;
    if (distancePID != null) forwardSpeed = distancePID.calculate(0.0, -distance);
    else forwardSpeed = distance * 0.5;

    // yaw PID => angular speed (deg -> rad)
    angleIntegral += yawErrorDeg * dtSeconds;
    double angleDerivative = (yawErrorDeg - lastAngleError) / Math.max(dtSeconds, 1e-6);
    lastAngleError = yawErrorDeg;
    double omegaDeg;
    if (yawPID != null) omegaDeg = yawPID.calculate(0.0, yawErrorDeg);
    else omegaDeg = yawErrorDeg * 0.02;
    double omega = Math.toRadians(omegaDeg);

    // Build world-frame velocity vector (forward component along angleToTarget)
    double vx_world = forwardSpeed * Math.cos(angleToTarget);
    double vy_world = forwardSpeed * Math.sin(angleToTarget);

    // Rotate world velocities into robot frame: [v_robot] = R(-yaw) * [v_world]
    double cos = Math.cos(-robotYawRad);
    double sin = Math.sin(-robotYawRad);
    double vx_robot = vx_world * cos - vy_world * sin; // forward
    double vy_robot = vx_world * sin + vy_world * cos; // strafe (right positive)

    // Clamp modestly to avoid huge speeds
    vx_robot = clamp(vx_robot, -2.0, 2.0);
    vy_robot = clamp(vy_robot, -2.0, 2.0);
    omega = clamp(omega, -Math.PI, Math.PI);

    return new double[]{vx_robot, vy_robot, omega};
  }

  // ==========================
  // LIMELIGHT STATUS
  // ==========================

  /** Returns the time since the last update (ms) */
  public long getTimeSinceLastUpdate() {
    return limelight.getTimeSinceLastUpdate();
  }

  /** Checks if Limelight is connected */
  public boolean isConnected() {
    return limelight.isConnected();
  }


  // ==========================
  // PYTHON SNAP METHODS
  // ==========================

  /** Updates Python SnapScript inputs (8 values) */
  public boolean updatePythonInputs(double i1, double i2, double i3, double i4,
                                    double i5, double i6, double i7, double i8) {
    return limelight.updatePythonInputs(i1, i2, i3, i4, i5, i6, i7, i8);
  }

  /** Updates Python SnapScript inputs using an array */
  public boolean updatePythonInputs(double[] inputs) {
    return limelight.updatePythonInputs(inputs);
  }


  // ==========================
  // PIPELINE METHODS
  // ==========================

  /** Switches to the desired pipeline by index */
  public boolean switchPipeline(int index) {
    return limelight.pipelineSwitch(index);
  }

  /** Reloads the current pipeline */
  public boolean reloadPipeline() {
    return limelight.reloadPipeline();
  }

  /** Captures a snapshot */
  public boolean captureSnapshot(String name) {
    return limelight.captureSnapshot(name);
  }

  /** Deletes a snapshot */
  public boolean deleteSnapshot(String name) {
    return limelight.deleteSnapshot(name);
  }


  // ==========================
  // FIELD / ORIENTATION METHODS
  // ==========================

  /** Updates robot orientation for MegaTag2 using the Pedro localizer heading (degrees). */
  private void updateRobotOrientation() {
    if (pedroLocalizer == null) return;
    PedroPose2d p = pedroLocalizer.getPose();
    if (p == null) return;
    double yawDeg = Math.toDegrees(p.heading);
    limelight.updateRobotOrientation(yawDeg);
  }

  /** Uploads a field map */
  public boolean uploadFieldmap(LLFieldMap map, Integer index) {
    return limelight.uploadFieldmap(map, index);
  }


  // ==========================
  // POLLING CONTROL
  // ==========================

  public void start() { limelight.start(); }
  public void pause() { limelight.pause(); }
  public void stop() { limelight.stop(); }
  public boolean isRunning() { return limelight.isRunning(); }

  /** Sets the polling rate in Hz */
  public void setPollRateHz(int hz) {
    limelight.setPollRateHz(hz);
  }


  // ==========================
  // LATENCY / PERFORMANCE INFO
  // ==========================

  /** Returns the capture latency (ms) */
  public double getCaptureLatency() {
    return limelight.getLatestResult().getCaptureLatency();
  }

  /** Returns the target processing latency (ms) */
  public double getTargetingLatency() {
    return limelight.getLatestResult().getTargetingLatency();
  }

  /** Returns the result parsing latency (ms) */
  public double getParseLatency() {
    return limelight.getLatestResult().getParseLatency();
  }


  // ==========================
  // FIDUCIAL UTILITIES
  // ==========================

  /** Returns the ID of the closest fiducial */
  public Integer getClosestFiducialID() {
    List<LLResultTypes.FiducialResult> fiducials = getFiducials();
    if (fiducials == null || fiducials.isEmpty()) return null;

    LLResultTypes.FiducialResult closest = fiducials.get(0);
    double minDist = getDistanceToFiducial(closest);
    for (LLResultTypes.FiducialResult f : fiducials) {
      double dist = getDistanceToFiducial(f);
      if (dist < minDist) {
        minDist = dist;
        closest = f;
      }
    }
    return closest.getFiducialId();
  }


  // ==========================
  // MOVEMENT SETUP & CONTROL
  // ==========================

  /** Configures the motors and provides PIDController instances for movement. */
  public void setupMovement(DcMotor leftMotor, DcMotor rightMotor, PIDController distancePID, PIDController yawPID) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.distancePID = distancePID;
    this.yawPID = yawPID;
  }

  /**
   * Moves the robot (tank drive) toward a fiducial until it reaches the desired distance.
   * @return true if target distance is reached
   */
  public boolean moveToFiducialByIdTank(int targetID, double desiredDistance, double maxPower, Telemetry telemetry) {
    if (leftMotor == null || rightMotor == null || distancePID == null || yawPID == null) return false;

    List<LLResultTypes.FiducialResult> fiducials = getFiducials();
    if (fiducials == null || fiducials.isEmpty()) return false;

    LLResultTypes.FiducialResult target = null;
    for (LLResultTypes.FiducialResult f : fiducials) {
      if (f.getFiducialId() == targetID) { target = f; break; }
    }

    double distance = getDistanceToFiducial(target);
    double distanceError = distance - desiredDistance;

    // Stop when within tolerance
    if (Math.abs(distanceError) < 0.05) {
      leftMotor.setPower(0);
      rightMotor.setPower(0);
      return true;
    }

    // PID forward
    double forwardPower = distancePID.calculate(desiredDistance, distanceError);
    forwardPower = clamp(forwardPower, -maxPower, maxPower);

    // PID yaw correction
    double yawToTag = getYawToFiducial(target);
    double yawCorrection = yawPID.calculate(0, yawToTag);

    // Apply motor power
    double leftPower = clamp(forwardPower + yawCorrection, -maxPower, maxPower);
    double rightPower = clamp(forwardPower - yawCorrection, -maxPower, maxPower);

    leftMotor.setPower(leftPower);
    rightMotor.setPower(rightPower);

    // Telemetry
    if (telemetry != null) {
      telemetry.addData("Fiducial ID", target.getFiducialId());
      telemetry.addData("Distance Error", distanceError);
      telemetry.addData("Forward Power", forwardPower);
      telemetry.addData("Yaw To Tag", yawToTag);
      telemetry.addData("Yaw Correction", yawCorrection);
      telemetry.addData("Left Power", leftPower);
      telemetry.addData("Right Power", rightPower);
      telemetry.update();
    }

    return false;
  }

  /**
   * Moves the robot (mecanum drive) toward a fiducial until it reaches the desired distance.
   * @return true if target distance is reached
   */
  public boolean moveToFiducialByIdMecanum(int targetID, DcMotor frontLeft, DcMotor frontRight,
                                           DcMotor backLeft, DcMotor backRight,
                                           double desiredDistance, double maxPower, Telemetry telemetry) {

    if (frontLeft == null || frontRight == null || backLeft == null || backRight == null ||
            distancePID == null || yawPID == null)
      return false;

    List<LLResultTypes.FiducialResult> fiducials = getFiducials();
    if (fiducials == null || fiducials.isEmpty()) return false;

    LLResultTypes.FiducialResult target = null;
    for (LLResultTypes.FiducialResult f : fiducials) {
      if (f.getFiducialId() == targetID) { target = f; break; }
    }
    if (target == null) return false;

    double distance = getDistanceToFiducial(target);
    double distanceError = distance - desiredDistance;
    double yawToTag = getYawToFiducial(target);

    // Stop when within tolerance
    if (Math.abs(distanceError) < 0.05) {
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
      return true;
    }

    // PID distance and yaw
    double forwardPower = clamp(distancePID.calculate(desiredDistance, distanceError), -maxPower, maxPower);
    double yawCorrection = yawPID.calculate(0, yawToTag);

    // Convert polar to Cartesian for mecanum drive
    double strafePower = forwardPower * Math.sin(Math.toRadians(yawToTag));
    double forwardComponent = forwardPower * Math.cos(Math.toRadians(yawToTag));

    // Calculate motor powers
    double fl = clamp(forwardComponent + strafePower + yawCorrection, -maxPower, maxPower);
    double fr = clamp(forwardComponent - strafePower - yawCorrection, -maxPower, maxPower);
    double bl = clamp(forwardComponent - strafePower + yawCorrection, -maxPower, maxPower);
    double br = clamp(forwardComponent + strafePower - yawCorrection, -maxPower, maxPower);

    // Apply power
    frontLeft.setPower(fl);
    frontRight.setPower(fr);
    backLeft.setPower(bl);
    backRight.setPower(br);

    // Telemetry
    if (telemetry != null) {
      telemetry.addData("Fiducial ID", target.getFiducialId());
      telemetry.addData("Distance Error", distanceError);
      telemetry.addData("Forward", forwardComponent);
      telemetry.addData("Strafe", strafePower);
      telemetry.addData("Yaw Correction", yawCorrection);
      telemetry.addData("FL", fl);
      telemetry.addData("FR", fr);
      telemetry.addData("BL", bl);
      telemetry.addData("BR", br);
      telemetry.update();
    }

    return false;
  }


  // ==========================
  // UTILITY
  // ==========================

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
