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
    private LLResult results;

    // Three 1D Kalman filters - one drift estimate per axis
    private DriftKalmanFilter xDriftFilter;
    private DriftKalmanFilter yDriftFilter;
    private DriftKalmanFilter headingDriftFilter;

    private boolean initialized = false;

    // Tuning parameters - measure these from your robot
    private static final double X_DRIFT_SIGMA = 0.01; // inches/sqrt(sec)
    private static final double Y_DRIFT_SIGMA = 0.01; // inches/sqrt(sec)
    private static final double HEADING_DRIFT_SIGMA = 0.001; // rad/sqrt(sec)

    private static final double LIMELIGHT_X_STD = 0.5; // inches
    private static final double LIMELIGHT_Y_STD = 0.5; // inches
    private static final double LIMELIGHT_HEADING_STD = 0.05; // radians

    private static final double LOOP_TIME = 0.02; // 20ms = 50Hz

    public Vision(HardwareMap hardwareMap, Follower follower){
        limelight = hardwareMap.get(Limelight3A.class, Constants.Vision.cameraName);
        this.follower = follower;
        limelight.pipelineSwitch(0); // Megatag 2 pipeline
        limelight.start();

        // Initialize drift filters
        xDriftFilter = new DriftKalmanFilter(X_DRIFT_SIGMA, LIMELIGHT_X_STD, LOOP_TIME);
        yDriftFilter = new DriftKalmanFilter(Y_DRIFT_SIGMA, LIMELIGHT_Y_STD, LOOP_TIME);
        headingDriftFilter = new DriftKalmanFilter(HEADING_DRIFT_SIGMA, LIMELIGHT_HEADING_STD, LOOP_TIME);
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

        // Predict step - runs every loop (only increases uncertainty)
        xDriftFilter.predict();
        yDriftFilter.predict();
        headingDriftFilter.predict();

        // Update step - only when vision is available
        if (results != null && results.isValid()) {
            Pose3D botPose = results.getBotpose_MT2();
            if (botPose != null) {
                // Get vision measurements (absolute position from field)
                double visionX = botPose.getPosition().x;
                double visionY = botPose.getPosition().y;
                double visionHeading = Math.toRadians(botPose.getOrientation().getYaw());

                // Get odometry readings (from Pinpoint via follower)
                Pose odometryPose = follower.getPose();
                double odometryX = odometryPose.getX();
                double odometryY = odometryPose.getY();
                double odometryHeading = odometryPose.getHeading();

                // Update drift estimates using vision and odometry
                xDriftFilter.update(visionX, odometryX);
                yDriftFilter.update(visionY, odometryY);
                headingDriftFilter.update(visionHeading, odometryHeading);

                // Calculate corrected pose: actual position = odometry - drift
                double correctedX = odometryX - xDriftFilter.getDrift();
                double correctedY = odometryY - yDriftFilter.getDrift();
                double correctedHeading = normalizeAngle(odometryHeading - headingDriftFilter.getDrift());

                // Update follower with drift-corrected pose
                follower.setPose(new Pose(correctedX, correctedY, correctedHeading));
            }
        }
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
     * Get raw vision pose measurements for tuning
     * @return [x, y, heading] from Limelight, or null if no valid reading
     */
    public double[] getVisionPose() {
        if (results != null && results.isValid()) {
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