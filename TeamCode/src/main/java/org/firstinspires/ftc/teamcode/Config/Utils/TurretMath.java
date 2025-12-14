package org.firstinspires.ftc.teamcode.Config.Utils;

import org.firstinspires.ftc.teamcode.Config.Constants;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.hardware.limelightvision.LLResult;

public class TurretMath {

  public static double TurretTrackPinpoint(double goalDeg, double RthetaDeg) {
    // Convert inputs to radians
    double goal = Math.toRadians(goalDeg);
    double Rtheta = Math.toRadians(RthetaDeg);

    // 1. Compute raw angle difference in radians
    double raw = goal - Rtheta;

    // 2. Wrap to [-π, π]
    double wrapped = Math.atan2(Math.sin(raw), Math.cos(raw));

    // 3. Convert turret limits to radians
    double minRad = Math.toRadians(Constants.Turret.minAngle); // e.g. -135°
    double maxRad = Math.toRadians(Constants.Turret.maxAngle); // e.g. +135°

    // 4. Clamp to nearest allowed angle
    double turretRad;
    if (wrapped >= minRad && wrapped <= maxRad) {
      turretRad = wrapped;
    } else {
      double toMin = Math.abs(wrapped - minRad);
      double toMax = Math.abs(wrapped - maxRad);
      turretRad = (toMin < toMax) ? minRad : maxRad;
    }

    // 5. Convert radians back to degrees for servo mapping
    double turretDeg = Math.toDegrees(turretRad);

    // 6. Convert degrees → servo position 0..1
    // minAngle → 0.0
    // maxAngle → 1.0
    return (turretDeg - Constants.Turret.minAngle)
        / (Constants.Turret.maxAngle - Constants.Turret.minAngle);
  }

  /**
   * @param result must make sure to validate result before use
   */
  public static double TurretTrackFromLimelight(LLResult result) {
    double knownGoalHeadingDeg = Constants.Turret.GoalAngle;

    double txDeg = result.getTx(); // Limelight horizontal offset (degrees)

    // New behavior: goal heading is fixed, LL tx is a correction
    double desiredTurretDeg = knownGoalHeadingDeg + txDeg;

    // Convert turret target to radians
    double desiredRad = Math.toRadians(desiredTurretDeg);

    // Wrap to [-π, π]
    double wrapped = Math.atan2(Math.sin(desiredRad), Math.cos(desiredRad));

    // Limits (in radians)
    double minRad = Math.toRadians(Constants.Turret.minAngle);
    double maxRad = Math.toRadians(Constants.Turret.maxAngle);

    // Clamp to nearest limit
    double turretRad;
    if (wrapped >= minRad && wrapped <= maxRad) {
      turretRad = wrapped;
    } else {
      double toMin = Math.abs(wrapped - minRad);
      double toMax = Math.abs(wrapped - maxRad);
      turretRad = (toMin < toMax) ? minRad : maxRad;
    }

    // Convert back to degrees for servo mapping
    double turretDeg = Math.toDegrees(turretRad);
    // Convert to servo position 0–1
    return (turretDeg - Constants.Turret.minAngle)
        / (Constants.Turret.maxAngle - Constants.Turret.minAngle);
  }

  public static double TurretTrackKalman(
      LLResult result, double robotHeadingDeg, double lastRobotHeadingDeg) {
    double goalHeadingDeg = Constants.Turret.GoalAngle;

    KalmanFilterParameters params =
        new KalmanFilterParameters(
            Constants.Turret.TURRET_KF_MODEL_COVARIANCE, // model covariance (robot heading is good)
            Constants.Turret.TURRET_KF_DATA_COVARIANCE // data covariance  (Limelight is noisier)
            );

    KalmanFilter kf =
        new KalmanFilter(
            params,
            Constants.Turret.TURRET_KF_START_STATE, // start turret angle (deg)
            Constants.Turret.TURRET_KF_START_VARIANCE, // variance (~4 deg std)
            Constants.Turret.TURRET_KF_START_GAIN);

    // -------------------------
    // 2) Robot heading delta (Pinpoint)
    // -------------------------
    double deltaRobotDeg = robotHeadingDeg - lastRobotHeadingDeg;

    // -------------------------
    // 3) Model update:
    //    Robot turning CW means turret target shifts CCW
    // -------------------------
    double modelDelta = -deltaRobotDeg;

    // -------------------------
    // 4) Measurement from Limelight
    //    Limelight tx already IS turret-relative error
    // -------------------------
    double measurementDeg;
    if (result != null && result.isValid()) {
      measurementDeg = result.getTx();
    } else {
      // no vision → don't correct
      measurementDeg = kf.getState();
    }

    // -------------------------
    // 5) Kalman update
    // -------------------------
    kf.update(modelDelta, measurementDeg);

    // -------------------------
    // 6) Get fused turret-relative angle
    // -------------------------
    double turretDeg = kf.getState();

    // -------------------------
    // 7) Add feedforward toward known goal heading
    //    (this centers the turret even without vision)
    // -------------------------
    double goalRelativeDeg = goalHeadingDeg - robotHeadingDeg;
    turretDeg += goalRelativeDeg;

    // -------------------------
    // 8) Wrap to [-180, 180]
    // -------------------------
    turretDeg = ((turretDeg + 180) % 360 + 360) % 360 - 180;

    // -------------------------
    // 9) Clamp to turret limits
    // -------------------------
    double minDeg = Constants.Turret.minAngle;
    double maxDeg = Constants.Turret.maxAngle;

    turretDeg = Math.max(minDeg, Math.min(maxDeg, turretDeg));

    // -------------------------
    // 10) Convert to servo position
    // -------------------------
    return (turretDeg - minDeg) / (maxDeg - minDeg);
  }
}
