package org.firstinspires.ftc.teamcode.Config.Utils;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Helper class for managing Limelight 3A camera and MT2 (MegaTag2) relocalization. Provides
 * easy-to-use methods for vision processing and pose estimation.
 */
public class LimelightHelper {
  private Limelight3A limelight;
  private String deviceName;

  // Pipeline indices
  private int mt2Pipeline = 0;
  private int detectorPipeline = 1;

  // MT2 relocalization settings
  private double confidenceThreshold = 0.5;
  private long lastValidPoseTime = 0;
  private Pose3D lastValidPose = null;

  public LimelightHelper(HardwareMap hardwareMap) {
    this(hardwareMap, "limelight");
  }

  public LimelightHelper(HardwareMap hardwareMap, String deviceName) {
    this.deviceName = deviceName;
    this.limelight = hardwareMap.get(Limelight3A.class, deviceName);
    limelight.pipelineSwitch(mt2Pipeline);
    limelight.start();
  }

  // === Pipeline Management ===

  public void enableMT2() {
    limelight.pipelineSwitch(mt2Pipeline);
  }

  public void enableDetector() {
    limelight.pipelineSwitch(detectorPipeline);
  }

  public void setPipelineIndices(int mt2Index, int detectorIndex) {
    this.mt2Pipeline = mt2Index;
    this.detectorPipeline = detectorIndex;
  }

  public void switchPipeline(int pipelineIndex) {
    limelight.pipelineSwitch(pipelineIndex);
  }

  public void updateIMUData(double yaw) {
    limelight.updateRobotOrientation(yaw);
  }

  // === MT2 Relocalization ===

  public Pose3D getMT2BotPose() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return null;
    }

    Pose3D botPose = result.getBotpose();

    if (botPose != null && getMT2Confidence() >= confidenceThreshold) {
      lastValidPose = botPose;
      lastValidPoseTime = System.currentTimeMillis();
    }

    return botPose;
  }

  public double getMT2Confidence() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return 0.0;
    }

    // Use target area as proxy for confidence
    double targetArea = result.getTa();
    if (targetArea <= 0) {
      return 0.0;
    }

    // Normalize to 0-1 range
    return Math.max(0.0, Math.min(1.0, targetArea / 10.0));
  }

  public int getMT2TagCount() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return 0;
    }

    // Count fiducial results (AprilTags)
    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
    if (fiducials != null) {
      return fiducials.size();
    }
    return 0;
  }

  public double getMT2AvgTagArea() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return 0.0;
    }
    return result.getTa();
  }

  public boolean hasMT2Data() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return false;
    }

    Pose3D pose = result.getBotpose();
    if (pose == null) {
      return false;
    }

    return getMT2TagCount() >= 1 && getMT2Confidence() >= confidenceThreshold;
  }

  public Pose3D getLastValidMT2Pose() {
    return lastValidPose;
  }

  public long getTimeSinceLastValidPose() {
    if (lastValidPose == null) {
      return -1;
    }
    return System.currentTimeMillis() - lastValidPoseTime;
  }

  public void setConfidenceThreshold(double threshold) {
    this.confidenceThreshold = Math.max(0.0, Math.min(1.0, threshold));
  }

  // === Target Detection ===

  public boolean hasTarget() {
    LLResult result = limelight.getLatestResult();
    return result != null && result.isValid() && result.getTx() != 0;
  }

  public double getTargetX() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return 0.0;
    }
    return result.getTx();
  }

  public double getTargetY() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return 0.0;
    }
    return result.getTy();
  }

  public double getTargetArea() {
    LLResult result = limelight.getLatestResult();
    if (result == null || !result.isValid()) {
      return 0.0;
    }
    return result.getTa();
  }

  public double getCaptureLatency() {
    LLResult result = limelight.getLatestResult();
    if (result == null) {
      return 0.0;
    }
    return result.getCaptureLatency();
  }

  public double getTargetLatency() {
    LLResult result = limelight.getLatestResult();
    if (result == null) {
      return 0.0;
    }
    return result.getTargetingLatency();
  }

  // === Raw Data Access ===

  public LLResult getLatestResult() {
    return limelight.getLatestResult();
  }

  public Limelight3A getLimelight() {
    return limelight;
  }

  // === Lifecycle Management ===

  public void start() {
    limelight.start();
  }

  public void stop() {
    limelight.stop();
  }

  public void shutdown() {
    limelight.shutdown();
  }
}
