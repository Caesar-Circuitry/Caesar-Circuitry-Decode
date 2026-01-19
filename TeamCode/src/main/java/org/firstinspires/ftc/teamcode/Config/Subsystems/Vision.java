package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Constants;

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
    private boolean enablePoseCorrection = true; // Flag to enable/disable pose correction


    public Vision(HardwareMap hardwareMap, Follower follower, Turret turret){
        limelight = hardwareMap.get(Limelight3A.class, Constants.Vision.cameraName);
        this.follower = follower;
        this.turret = turret;
        limelight.pipelineSwitch(0); // Megatag 2 pipeline
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

        // Get robot odometry
        Pose odometryPose = follower.getPose();
        double robotHeading = odometryPose.getHeading();

        // Calculate camera heading (robot heading + turret angle)
        double turretAngleRad = Math.toRadians(turret.getCurrentTurretAngle());
        double cameraHeading = robotHeading + turretAngleRad;

        // Update Limelight with camera's orientation for MT2
        // This ensures MT2 uses the correct heading to resolve AprilTag ambiguity
        limelight.updateRobotOrientation(Math.toDegrees(cameraHeading));

        // Predict step - runs every loop (only increases uncertainty)
        xDriftFilter.predict();
        yDriftFilter.predict();
        headingDriftFilter.predict();

        // Update step - only when vision is available
        if (results != null && results.isValid()) {
            // MT2 now uses the camera heading we just set
            Pose3D botPose = results.getBotpose_MT2();

            if (botPose != null) {
                // Get vision measurements (camera position from field)
                double limelightX = botPose.getPosition().x;
                double limelightY = botPose.getPosition().y;
                double limelightHeading = Math.toRadians(botPose.getOrientation().getYaw());

                // Transform vision measurements from turret-mounted camera to robot center
                double[] robotPose = transformLimelightToRobot(limelightX, limelightY, limelightHeading);
                double visionX = robotPose[0];
                double visionY = robotPose[1];
                double visionHeading = robotPose[2];

                // Get odometry readings (from Pinpoint via follower)
                double odometryX = odometryPose.getX();
                double odometryY = odometryPose.getY();
                double odometryHeading = odometryPose.getHeading();

                // Update drift estimates using vision and odometry
                xDriftFilter.update(visionX, odometryX);
                yDriftFilter.update(visionY, odometryY);
                headingDriftFilter.update(visionHeading, odometryHeading);

                // Only update follower pose if pose correction is enabled
                if (enablePoseCorrection) {
                    // Calculate corrected pose: actual position = odometry - drift
                    double correctedX = odometryX - xDriftFilter.getDrift();
                    double correctedY = odometryY - yDriftFilter.getDrift();
                    double correctedHeading = normalizeAngle(odometryHeading - headingDriftFilter.getDrift());

                    // Update follower with drift-corrected pose
                    follower.setPose(new Pose(correctedX, correctedY, correctedHeading));
                }
            }
        }
    }

    /**
     * Transform Limelight pose (camera position) to robot center pose
     * Accounts for turret rotation and camera offset
     *
     * @param limelightX Camera X position from Limelight (field coordinates)
     * @param limelightY Camera Y position from Limelight (field coordinates)
     * @param limelightHeading Camera heading from Limelight (field coordinates)
     * @return [robotX, robotY, robotHeading] in field coordinates
     */
    private double[] transformLimelightToRobot(double limelightX, double limelightY, double limelightHeading) {
        // Get current turret angle (robot-relative, in degrees)
        double turretAngleDeg = turret.getCurrentTurretAngle();
        double turretAngleRad = Math.toRadians(turretAngleDeg);

        // Get robot heading from odometry
        double robotHeading = follower.getPose().getHeading();

        // Calculate field-relative turret angle
        double fieldTurretAngle = robotHeading + turretAngleRad;

        // Limelight offset from robot center (rotated by turret + robot heading)
        double offsetX = Constants.Vision.LIMELIGHT_X_OFFSET;
        double offsetY = Constants.Vision.LIMELIGHT_Y_OFFSET;

        // Rotate offset by field-relative turret angle
        double rotatedOffsetX = offsetX * Math.cos(fieldTurretAngle) - offsetY * Math.sin(fieldTurretAngle);
        double rotatedOffsetY = offsetX * Math.sin(fieldTurretAngle) + offsetY * Math.cos(fieldTurretAngle);

        // Robot center is camera position minus the rotated offset
        double robotX = limelightX - rotatedOffsetX;
        double robotY = limelightY - rotatedOffsetY;

        // Robot heading: Limelight heading minus turret angle minus heading offset
        double robotHeadingFromVision = limelightHeading - turretAngleRad - Math.toRadians(Constants.Vision.LIMELIGHT_HEADING_OFFSET);

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
            // MT2 automatically uses orientation from Pinpoint
            Pose3D botPose = results.getBotpose_MT2();
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
     * Enable or disable automatic pose correction
     * Disable during tuning to prevent feedback loops
     * @param enable true to enable pose correction, false to disable
     */
    public void setEnablePoseCorrection(boolean enable) {
        this.enablePoseCorrection = enable;
    }

    /**
     * Get raw vision pose measurements for tuning
     * @return [x, y, heading] from Limelight, or null if no valid reading
     */
    public double[] getVisionPose() {
        if (results != null && results.isValid()) {
            // MT2 automatically uses orientation from Pinpoint
            Pose3D botPose = results.getBotpose_MT2();
            if (botPose != null) {
                return new double[]{
                    botPose.getPosition().x,
                    botPose.getPosition().y,
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
                return new double[]{
                    botPose.getPosition().x,
                    botPose.getPosition().y,
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

        public DriftKalmanFilter(double driftSigma, double visionStd, double dt) {
            this.drift = 0.0; // Start assuming no drift
            this.P = 0.5; // Initial uncertainty (setup accuracy)
            this.Q = driftSigma * driftSigma * dt; // Random walk variance
            this.R = visionStd * visionStd; // Vision measurement variance
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
         * @param visionMeasurement - absolute position from Limelight
         * @param odometryReading   - position from Pinpoint
         */
        public void update(double visionMeasurement, double odometryReading) {
            // Innovation: difference between vision and (odometry - current drift estimate)
            // y = z - (x_odo - x_drift)
            double y = visionMeasurement - (odometryReading - drift);

            // Innovation covariance: S = P + R
            double S = P + R;

            // Kalman gain: K = P / S
            double K = P / S;

            // Update drift estimate: drift = drift + K * y
            drift = drift + K * y;

            // Update uncertainty: P = (1 - K) * P
            P = (1 - K) * P;
        }

        public double getDrift() {
            return drift;
        }
    }
}