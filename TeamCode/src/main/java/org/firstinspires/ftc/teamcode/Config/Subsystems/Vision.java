package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import java.util.LinkedList;
import java.util.List;

public class Vision extends WSubsystem {
    //see this article for EKF https://medium.com/@vikramaditya.nishant/how-to-make-a-zero-drift-ftc-localizer-with-kalman-filters-911807e0916d
    private Limelight3A limelight;
    private Follower follower;
    private Turret turret;
    private LLResult results;

    // Three 1D Kalman filters - one drift estimate per axis
    private DriftKalmanFilter xDriftFilter;
    private DriftKalmanFilter yDriftFilter;
    private DriftKalmanFilter headingDriftFilter;

    private boolean initialized = false;
    private boolean enablePoseCorrection = false; // Disabled by default - enable once Limelight field map is configured

    private LinkedList<TelemetryPacket> telemetryPackets = new LinkedList<>();


    public Vision(HardwareMap hardwareMap, Follower follower, Turret turret){
        limelight = hardwareMap.get(Limelight3A.class, Constants.Vision.cameraName);
        this.follower = follower;
        this.turret = turret;
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        // Initialize drift filters using constants from Constants.Vision
        xDriftFilter = new DriftKalmanFilter(Constants.Vision.X_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_X_STD, Constants.Vision.LOOP_TIME);
        yDriftFilter = new DriftKalmanFilter(Constants.Vision.Y_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_Y_STD, Constants.Vision.LOOP_TIME);
        headingDriftFilter = new DriftKalmanFilter(Constants.Vision.HEADING_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_HEADING_STD, Constants.Vision.LOOP_TIME);
    }

    @Override
    public void read() {
        results = limelight.getLatestResult();
    }

    @Override
    public void loop() {
        if (!initialized) {
            initialized = true;
        }

        // Sanity check: reset drift filters if values are unreasonable
        // This catches accumulated garbage from previous bad readings
        double maxReasonableDrift = 36.0; // 3 feet
        double maxReasonableHeadingDrift = Math.PI; // 180 degrees max
        if (Math.abs(xDriftFilter.getDrift()) > maxReasonableDrift ||
            Math.abs(yDriftFilter.getDrift()) > maxReasonableDrift ||
            Math.abs(headingDriftFilter.getDrift()) > maxReasonableHeadingDrift) {
            resetDriftFilters();
        }

        // Get robot odometry
        Pose odometryPose = follower.getPose();
        double robotHeading = odometryPose.getHeading();

        // Calculate camera heading (robot heading + turret angle)
        double turretAngleRad = Math.toRadians(turret.getCurrentTurretAngle());
        double cameraHeading = robotHeading + turretAngleRad;

        // Update Limelight with camera's orientation for MT2
        limelight.updateRobotOrientation(Math.toDegrees(cameraHeading));

        // Predict step - runs every loop (only increases uncertainty)
        xDriftFilter.predict();
        yDriftFilter.predict();
        headingDriftFilter.predict();

        Double visionX = null, visionY = null, visionHeading = null;
        Double correctedX = null, correctedY = null, correctedHeading = null;
        boolean visionUsed = false;
        int numTagsDetected = 0;

        // Update step - only when vision is available and passes quality checks
        if (results != null && results.isValid()) {
            // Use MT1 for absolute positioning (doesn't require odometry input)
            Pose3D botPose = results.getBotpose();

            // Quality checks
            List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
            numTagsDetected = fiducials != null ? fiducials.size() : 0;
            boolean passesQualityCheck = fiducials != null && fiducials.size() >= Constants.Vision.MIN_TAGS_FOR_CORRECTION;

            // Check tag distance if we have fiducials
            if (passesQualityCheck && fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    // Skip if any tag is too far away
                    double tagDist = fiducial.getTargetPoseCameraSpace().getPosition().z * 39.3701; // meters to inches
                    if (tagDist > Constants.Vision.MAX_TAG_DISTANCE) {
                        passesQualityCheck = false;
                        break;
                    }
                }
            }

            if (botPose != null && passesQualityCheck) {
                // Validate raw meter values - FTC field is ~3.66m x 3.66m (144 inches)
                // Limelight uses field center as origin, so valid range is -1.83m to +1.83m
                // Allow some margin: -2.5m to +2.5m
                double rawXMeters = botPose.getPosition().x;
                double rawYMeters = botPose.getPosition().y;

                boolean rawValuesValid = rawXMeters >= -2.5 && rawXMeters <= 2.5 &&
                                          rawYMeters >= -2.5 && rawYMeters <= 2.5;

                // Only process if raw values are within valid range
                if (rawValuesValid) {
                    // Limelight returns position in meters with field center as origin (0,0,0)
                    // Convert to inches and shift to corner origin (add 72 = half of 144 inch field)
                    // NOTE: Limelight X = Pedro Y, Limelight Y = Pedro X (coordinate system swap)
                    double limelightX = rawYMeters * 39.3701 + 72.0; // Limelight Y -> Pedro X
                    double limelightY = rawXMeters * 39.3701 + 144.0; // Limelight X -> Pedro Y
                    double limelightHeading = Math.toRadians(botPose.getOrientation().getYaw());

                    double[] robotPose = transformLimelightToRobot(limelightX, limelightY, limelightHeading);
                    visionX = robotPose[0];
                    visionY = robotPose[1];
                    visionHeading = robotPose[2];

                    // Sanity check: reject poses outside the field (with margin for error)
                    // Field is 144x144 inches, allow some margin for edge cases
                    boolean poseInBounds = visionX >= -10 && visionX <= 154 &&
                                           visionY >= -10 && visionY <= 154;

                    if (!poseInBounds) {
                        // Invalid pose detected - skip this measurement
                        visionX = null;
                        visionY = null;
                        visionHeading = null;
                    } else {
                        double odometryX = odometryPose.getX();
                        double odometryY = odometryPose.getY();
                        double odometryHeading = odometryPose.getHeading();

                        // Always update X/Y drift (reliable with 1+ tags)
                        xDriftFilter.update(visionX, odometryX);
                        yDriftFilter.update(visionY, odometryY);

                        // Only update heading drift when 2+ tags are visible
                        // MT1 heading with single tag is very unreliable
                        if (numTagsDetected >= 2) {
                            headingDriftFilter.updateAngle(visionHeading, odometryHeading);
                        }
                        visionUsed = true;

                        if (enablePoseCorrection) {
                            correctedX = odometryX - xDriftFilter.getDrift();
                            correctedY = odometryY - yDriftFilter.getDrift();

                            // Only apply heading correction if we have 2+ tags
                            // Otherwise trust Pinpoint odometry heading (more reliable)
                            if (numTagsDetected >= 2) {
                                correctedHeading = normalizeAngle(odometryHeading - headingDriftFilter.getDrift());
                            } else {
                                correctedHeading = odometryHeading; // Trust odometry heading
                            }

                            // Final safety check before applying pose
                            // Only apply if corrected values are within field bounds
                            if (correctedX >= -10 && correctedX <= 154 &&
                                correctedY >= -10 && correctedY <= 154 &&
                                !Double.isNaN(correctedX) && !Double.isNaN(correctedY) &&
                                !Double.isInfinite(correctedX) && !Double.isInfinite(correctedY)) {
                                // Apply corrected pose (heading only corrected with 2+ tags)
                                follower.setPose(new Pose(correctedX, correctedY, correctedHeading));
                            }
                        }
                    }
                }
            }
        }

        // Telemetry logging
        if (Constants.Vision.logTelemetry) {
            telemetryPackets.clear();

            // Configuration & flags
            telemetryPackets.add(new TelemetryPacket("PoseCorrectionEnabled", enablePoseCorrection));
            telemetryPackets.add(new TelemetryPacket("Vision Used This Cycle", visionUsed));

            // Odometry
            telemetryPackets.add(new TelemetryPacket("Odo X", odometryPose.getX()));
            telemetryPackets.add(new TelemetryPacket("Odo Y", odometryPose.getY()));
            telemetryPackets.add(new TelemetryPacket("Odo Heading(rad)", robotHeading));

            // Turret & camera orientation
            telemetryPackets.add(new TelemetryPacket("Turret Angle(deg)", turret.getCurrentTurretAngle()));
            telemetryPackets.add(new TelemetryPacket("Camera Heading(deg)", Math.toDegrees(cameraHeading)));

            // Raw vision (MT1)
            telemetryPackets.add(new TelemetryPacket("Vision Valid", results != null && results.isValid()));
            if (results != null && results.isValid()) {
                Pose3D mt1Pose = results.getBotpose();
                List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
                telemetryPackets.add(new TelemetryPacket("Tags Detected", fiducials != null ? fiducials.size() : 0));
                if (mt1Pose != null) {
                    telemetryPackets.add(new TelemetryPacket("MT1 X (m)", mt1Pose.getPosition().x));
                    telemetryPackets.add(new TelemetryPacket("MT1 Y (m)", mt1Pose.getPosition().y));
                    telemetryPackets.add(new TelemetryPacket("MT1 Heading(deg)", mt1Pose.getOrientation().getYaw()));
                    // Show if raw values are in valid range (center origin: -1.83m to +1.83m)
                    boolean rawValid = mt1Pose.getPosition().x >= -2.5 && mt1Pose.getPosition().x <= 2.5 &&
                                       mt1Pose.getPosition().y >= -2.5 && mt1Pose.getPosition().y <= 2.5;
                    telemetryPackets.add(new TelemetryPacket("Raw Values Valid", rawValid));
                }
            }

            // Transformed robot pose from LL
            if (visionX != null) {
                telemetryPackets.add(new TelemetryPacket("Vision->Robot X", visionX));
                telemetryPackets.add(new TelemetryPacket("Vision->Robot Y", visionY));
                telemetryPackets.add(new TelemetryPacket("Vision->Robot Heading(rad)", visionHeading));
            }

            // Drift estimates
            telemetryPackets.add(new TelemetryPacket("Drift X", xDriftFilter.getDrift()));
            telemetryPackets.add(new TelemetryPacket("Drift Y", yDriftFilter.getDrift()));
            telemetryPackets.add(new TelemetryPacket("Drift Heading(rad)", headingDriftFilter.getDrift()));

            // Corrected pose
            if (correctedX != null) {
                telemetryPackets.add(new TelemetryPacket("Corrected X", correctedX));
                telemetryPackets.add(new TelemetryPacket("Corrected Y", correctedY));
                telemetryPackets.add(new TelemetryPacket("Corrected Heading(rad)", correctedHeading));
            }
        }
    }

    /**
     * Transform Limelight pose (camera position) to robot center pose
     * Accounts for turret rotation and camera offset
     *
     * MT1 returns the CAMERA's position and heading in field coordinates.
     * We need to find the ROBOT CENTER by subtracting the camera offset.
     *
     * @param limelightX Camera X position from Limelight (field coordinates, inches)
     * @param limelightY Camera Y position from Limelight (field coordinates, inches)
     * @param limelightHeading Camera heading from Limelight (field coordinates, radians)
     * @return [robotX, robotY, robotHeading] in field coordinates
     */
    private double[] transformLimelightToRobot(double limelightX, double limelightY, double limelightHeading) {
        // Get current turret angle (robot-relative, in degrees)
        double turretAngleDeg = turret.getCurrentTurretAngle();
        double turretAngleRad = Math.toRadians(turretAngleDeg);

        // The camera heading from Limelight IS the field-relative camera heading
        // Robot heading = Camera heading - turret angle - any mounting offset
        double robotHeadingFromVision = limelightHeading - turretAngleRad - Math.toRadians(Constants.Vision.LIMELIGHT_HEADING_OFFSET);

        // Camera offset from robot center (in robot-relative coordinates)
        // These are the offset when turret is at 0 degrees
        double offsetX = Constants.Vision.LIMELIGHT_X_OFFSET;
        double offsetY = Constants.Vision.LIMELIGHT_Y_OFFSET;

        // The camera is on the turret, so we need to rotate the offset by:
        // 1. The turret angle (where the turret is pointing relative to robot)
        // 2. The robot heading (to get field-relative offset)
        // Combined: rotate by (robotHeading + turretAngle) = cameraHeading in field
        double cameraFieldHeading = robotHeadingFromVision + turretAngleRad;

        // Rotate offset to field coordinates
        double rotatedOffsetX = offsetX * Math.cos(cameraFieldHeading) - offsetY * Math.sin(cameraFieldHeading);
        double rotatedOffsetY = offsetX * Math.sin(cameraFieldHeading) + offsetY * Math.cos(cameraFieldHeading);

        // Robot center = Camera position - rotated offset
        double robotX = limelightX - rotatedOffsetX;
        double robotY = limelightY - rotatedOffsetY;

        return new double[]{robotX, robotY, robotHeadingFromVision};
    }

    @Override
    public void write() {
        // No actuators
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getRangeFromTag(){
        if (results != null && results.isValid()) {
            // Use MT1 for range calculation
            Pose3D botPose = results.getBotpose();
            if (botPose != null) {
                return Math.sqrt(
                        Math.pow(botPose.getPosition().x, 2) +
                                Math.pow(botPose.getPosition().y, 2)
                );
            }
        }
        return 0.0;
    }
    public void readMotif(){
        if (results == null || !results.isValid()) {
            return;
        }

        // Get all detected AprilTags
        List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();

            // Check if tag is one of the motif tags (21, 22, 23)
            switch (tagId) {
                case 21:
                    Constants.Robot.CurrentMOTIF = Constants.Robot.motif.GPP;
                    break;
                case 22:
                    Constants.Robot.CurrentMOTIF = Constants.Robot.motif.PGP;
                    break;
                case 23:
                    Constants.Robot.CurrentMOTIF = Constants.Robot.motif.PPG;
                    break;
            }
        }
    }


    public double[] getDriftEstimates() {
        return new double[]{
                xDriftFilter.getDrift(),
                yDriftFilter.getDrift(),
                headingDriftFilter.getDrift()
        };
    }

    /**
     * Reset all drift filters to zero
     * Call this when drift estimates have become unreasonable
     */
    public void resetDriftFilters() {
        xDriftFilter = new DriftKalmanFilter(Constants.Vision.X_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_X_STD, Constants.Vision.LOOP_TIME);
        yDriftFilter = new DriftKalmanFilter(Constants.Vision.Y_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_Y_STD, Constants.Vision.LOOP_TIME);
        headingDriftFilter = new DriftKalmanFilter(Constants.Vision.HEADING_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_HEADING_STD, Constants.Vision.LOOP_TIME);
    }

    /**
     * Enable or disable automatic pose correction
     * Disable during tuning to prevent feedback loops
     * @param enable true to enable pose correction, false to disable
     */
    public void setEnablePoseCorrection(boolean enable) {
        this.enablePoseCorrection = enable;
    }

    /**
     * Set the robot's initial pose using MT1 vision
     * This reads the current MT1 pose and sets it as the follower's pose
     * Use this during init to establish starting position from AprilTags
     * @return true if pose was successfully set, false if no valid vision data
     */
    public boolean setInitialPoseFromVision() {
        // Make sure we have fresh data
        read();

        if (results != null && results.isValid()) {
            Pose3D botPose = results.getBotpose();
            if (botPose != null) {
                // Validate raw meter values (center origin: -1.83m to +1.83m with margin)
                double rawXMeters = botPose.getPosition().x;
                double rawYMeters = botPose.getPosition().y;

                boolean rawValuesValid = rawXMeters >= -2.5 && rawXMeters <= 2.5 &&
                                          rawYMeters >= -2.5 && rawYMeters <= 2.5;

                if (rawValuesValid) {
                    // Convert to inches with corner origin (add 72 for field center offset)
                    // NOTE: Limelight X = Pedro Y, Limelight Y = Pedro X (coordinate system swap)
                    double limelightX = rawYMeters * 39.3701 + 72.0; // Limelight Y -> Pedro X
                    double limelightY = rawXMeters * 39.3701 + 72.0; // Limelight X -> Pedro Y
                    double limelightHeading = Math.toRadians(botPose.getOrientation().getYaw());

                    // Transform from camera pose to robot center pose
                    double[] robotPose = transformLimelightToRobot(limelightX, limelightY, limelightHeading);

                    // Validate transformed pose is within field bounds
                    if (robotPose[0] >= -10 && robotPose[0] <= 154 &&
                        robotPose[1] >= -10 && robotPose[1] <= 154) {

                        follower.setPose(new Pose(robotPose[0], robotPose[1], robotPose[2]));

                        // Reset drift filters since we just set a new known position
                        resetDriftFilters();

                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * Command to set initial pose from vision
     * Useful for autonomous init - keeps trying until successful or timeout
     * @return InstantCommand that attempts to set pose from vision
     */
    public com.seattlesolvers.solverslib.command.InstantCommand setInitialPose() {
        return new com.seattlesolvers.solverslib.command.InstantCommand(this::setInitialPoseFromVision);
    }

    /**
     * Get raw vision pose measurements for tuning
     * @return [x, y, heading] from Limelight, or null if no valid reading
     */
    public double[] getVisionPose() {
        if (results != null && results.isValid()) {
            // Use MT1 for absolute positioning
            Pose3D botPose = results.getBotpose();
            if (botPose != null) {
                // Limelight returns position in meters with center origin
                // Convert to inches and add 72 for corner origin
                // NOTE: Limelight X = Pedro Y, Limelight Y = Pedro X (coordinate system swap)
                return new double[]{
                    botPose.getPosition().y * 39.3701 + 72.0, // Limelight Y -> Pedro X
                    botPose.getPosition().x * 39.3701 + 72.0, // Limelight X -> Pedro Y
                    Math.toRadians(botPose.getOrientation().getYaw())
                };
            }
        }
        return null;
    }

    /**
     * Get absolute pose from Megatag 1 (MT1) for initialization
     * MT1 doesn't require odometry input - good for initial pose setup
     * @return [x, y, heading] from Limelight MT1, or null if no valid reading
     */
    public double[] getMT1Pose() {
        if (results != null && results.isValid()) {
            // Use MT1 which doesn't require odometry input
            Pose3D botPose = results.getBotpose();
            if (botPose != null) {
                // Limelight returns position in meters with center origin
                // Convert to inches and add 72 for corner origin
                // NOTE: Limelight X = Pedro Y, Limelight Y = Pedro X (coordinate system swap)
                return new double[]{
                    botPose.getPosition().y * 39.3701 + 72.0, // Limelight Y -> Pedro X
                    botPose.getPosition().x * 39.3701 + 72.0, // Limelight X -> Pedro Y
                    Math.toRadians(botPose.getOrientation().getYaw())
                };
            }
        }
        return null;
    }

    /**
     * 1D Kalman Filter that estimates only drift (not position)
     * Following the simplified approach from the article
     */
    private static class DriftKalmanFilter {
        private double drift; // Current drift estimate (state)
        private double P; // Uncertainty in drift estimate (covariance)
        private final double Q; // Process noise (how much drift changes)
        private final double R; // Measurement noise (vision uncertainty)
        private final double maxDrift; // Maximum allowed drift value

        public DriftKalmanFilter(double driftSigma, double visionStd, double dt) {
            this(driftSigma, visionStd, dt, 36.0); // Default max drift of 36 inches
        }

        public DriftKalmanFilter(double driftSigma, double visionStd, double dt, double maxDrift) {
            this.drift = 0.0; // Start assuming no drift
            this.P = 0.5; // Initial uncertainty (setup accuracy)
            this.Q = driftSigma * driftSigma * dt; // Random walk variance
            this.R = visionStd * visionStd; // Vision measurement variance
            this.maxDrift = maxDrift;
        }

        /**
         * Predict step: drift doesn't change, only uncertainty increases
         * Equation: P = P + Q
         */
        public void predict() {
            P = P + Q;
        }

        /**
         * Update step: correct drift estimate using vision measurement
         *
         * Drift is defined as: drift = odometry - truth
         * So: truth = odometry - drift
         *
         * When we get a vision measurement (which approximates truth):
         * measured_drift = odometry - vision
         *
         * @param visionMeasurement - absolute position from Limelight (approximates truth)
         * @param odometryReading   - position from Pinpoint
         */
        public void update(double visionMeasurement, double odometryReading) {
            // Measured drift from this vision reading
            double measuredDrift = odometryReading - visionMeasurement;

            // Innovation: difference between measured drift and current drift estimate
            double y = measuredDrift - drift;

            // Innovation covariance: S = P + R
            double S = P + R;

            // Kalman gain: K = P / S
            double K = P / S;

            // Update drift estimate: drift = drift + K * y
            drift = drift + K * y;

            // Clamp drift to reasonable bounds
            drift = Math.max(-maxDrift, Math.min(maxDrift, drift));

            // Update uncertainty: P = (1 - K) * P
            P = (1 - K) * P;
        }

        /**
         * Update step for angular values - handles wrapping around ±π
         *
         * @param visionMeasurement - absolute angle from Limelight (radians)
         * @param odometryReading   - angle from Pinpoint (radians)
         */
        public void updateAngle(double visionMeasurement, double odometryReading) {
            // Measured drift from this vision reading, normalized
            double measuredDrift = normalizeAngleStatic(odometryReading - visionMeasurement);

            // Innovation: difference between measured drift and current drift estimate, normalized
            double y = normalizeAngleStatic(measuredDrift - drift);

            // Innovation covariance: S = P + R
            double S = P + R;

            // Kalman gain: K = P / S
            double K = P / S;

            // Update drift estimate: drift = drift + K * y
            drift = drift + K * y;

            // Normalize drift to [-π, π]
            drift = normalizeAngleStatic(drift);

            // Update uncertainty: P = (1 - K) * P
            P = (1 - K) * P;
        }

        private static double normalizeAngleStatic(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        public double getDrift() {
            return drift;
        }
    }

    @Override
    public LinkedList<TelemetryPacket> getTelemetry() {
        return telemetryPackets;
    }
}

