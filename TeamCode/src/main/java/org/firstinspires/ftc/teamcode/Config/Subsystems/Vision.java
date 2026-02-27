package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
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
    private final Limelight3A limelight;
    private final Follower follower;
    private final Turret turret;
    private LLResult results;

    // Three 1D Kalman filters - one drift estimate per axis
    private DriftKalmanFilter xDriftFilter;
    private DriftKalmanFilter yDriftFilter;
    private DriftKalmanFilter headingDriftFilter;

    private boolean enablePoseCorrection = false;

    private final LinkedList<TelemetryPacket> telemetryPackets = new LinkedList<>();

    public Vision(HardwareMap hardwareMap, Follower follower, Turret turret) {
        limelight = hardwareMap.get(Limelight3A.class, Constants.Vision.cameraName);
        this.follower = follower;
        this.turret = turret;
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        xDriftFilter = new DriftKalmanFilter(Constants.Vision.X_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_X_STD, Constants.Vision.LOOP_TIME);
        yDriftFilter = new DriftKalmanFilter(Constants.Vision.Y_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_Y_STD, Constants.Vision.LOOP_TIME);
        headingDriftFilter = new DriftKalmanFilter(Constants.Vision.HEADING_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_HEADING_STD, Constants.Vision.LOOP_TIME);
    }

    /**
     * Helper: get the current follower pose in FTC coordinates.
     * All internal Vision math uses FTC coordinates.
     */
    private Pose getOdometryFTC() {
        return follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
    }

    /**
     * Helper: set the follower pose from an FTC-coordinate pose.
     * Converts FTC -> Pedro before writing.
     */
    private void setFollowerPoseFromFTC(Pose ftcPose) {
        follower.setPose(ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
    }

    @Override
    public void read() {
        results = limelight.getLatestResult();
    }

    @Override
    public void loop() {
        // Sanity check: reset drift filters if values are unreasonable
        double maxReasonableDrift = 36.0;
        double maxReasonableHeadingDrift = Math.PI;
        if (Math.abs(xDriftFilter.getDrift()) > maxReasonableDrift ||
            Math.abs(yDriftFilter.getDrift()) > maxReasonableDrift ||
            Math.abs(headingDriftFilter.getDrift()) > maxReasonableHeadingDrift) {
            resetDriftFilters();
        }

        // All internal math in FTC coordinates
        Pose odometryPose = getOdometryFTC();
        double robotHeading = odometryPose.getHeading();

        // Turret angle for telemetry and camera-to-robot transform
        double turretAngleRad = Math.toRadians(turret.getCurrentTurretAngle());
        double cameraHeading = robotHeading + turretAngleRad;
        // Feed camera's actual field heading for MT2 (Limelight treats this as "robot" heading)
        limelight.updateRobotOrientation(Math.toDegrees(cameraHeading));

        // Predict step - runs every loop (only increases uncertainty)
        xDriftFilter.predict();
        yDriftFilter.predict();
        headingDriftFilter.predict();

        Double visionX = null, visionY = null, visionHeading = null;
        Double correctedX = null, correctedY = null, correctedHeading = null;
        boolean visionUsed = false;
        boolean usingMT2 = false;
        int numTagsDetected = 0;

        if (results != null && results.isValid()) {
            // Try MT2 first (heading-constrained, more accurate with good heading input).
            // Feed camera heading via updateRobotOrientation so Limelight constrains to that.
            // Fall back to MT1 if MT2 returns null or out-of-bounds data.
            Pose3D botPose = results.getBotpose_MT2();
            usingMT2 = true;

            // Validate MT2 result - reject if null or clearly outside field bounds
            if (botPose == null ||
                Math.abs(botPose.getPosition().x) > 2.5 ||
                Math.abs(botPose.getPosition().y) > 2.5) {
                botPose = results.getBotpose();
                usingMT2 = false;
            }

            List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
            numTagsDetected = fiducials != null ? fiducials.size() : 0;
            boolean passesQualityCheck = fiducials != null && fiducials.size() >= Constants.Vision.MIN_TAGS_FOR_CORRECTION;

            if (passesQualityCheck) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    double tagDist = fiducial.getTargetPoseCameraSpace().getPosition().z * 39.3701;
                    if (tagDist > Constants.Vision.MAX_TAG_DISTANCE) {
                        passesQualityCheck = false;
                        break;
                    }
                }
            }

            if (botPose != null && passesQualityCheck) {
                // Limelight already returns FTC coordinates - just convert meters to inches
                Pose cameraPose = getCameraPoseFromLimelight(botPose);

                if (cameraPose != null) {
                    // Transform camera pose to robot center (all in FTC coordinates)
                    Pose robotPose = transformCameraToRobot(cameraPose);

                    visionX = robotPose.getX();
                    visionY = robotPose.getY();
                    visionHeading = robotPose.getHeading();

                    // FTC field bounds: center origin, ±72 inches
                    boolean poseInBounds = visionX >= -82 && visionX <= 82 &&
                                           visionY >= -82 && visionY <= 82;

                    if (!poseInBounds) {
                        visionX = null;
                        visionY = null;
                        visionHeading = null;
                    } else {
                        double odometryX = odometryPose.getX();
                        double odometryY = odometryPose.getY();
                        double odometryHeadingVal = odometryPose.getHeading();

                        xDriftFilter.update(visionX, odometryX);
                        yDriftFilter.update(visionY, odometryY);

                        if (numTagsDetected >= 2) {
                            headingDriftFilter.updateAngle(visionHeading, odometryHeadingVal);
                        }
                        visionUsed = true;

                        if (enablePoseCorrection) {
                            correctedX = odometryX - xDriftFilter.getDrift();
                            correctedY = odometryY - yDriftFilter.getDrift();

                            if (numTagsDetected >= 2) {
                                correctedHeading = normalizeAngle(odometryHeadingVal - headingDriftFilter.getDrift());
                            } else {
                                correctedHeading = odometryHeadingVal;
                            }

                            if (correctedX >= -82 && correctedX <= 82 &&
                                correctedY >= -82 && correctedY <= 82 &&
                                !Double.isNaN(correctedX) && !Double.isNaN(correctedY) &&
                                !Double.isInfinite(correctedX) && !Double.isInfinite(correctedY)) {
                                // Convert FTC corrected pose back to Pedro for the follower
                                setFollowerPoseFromFTC(new Pose(correctedX, correctedY, correctedHeading));
                            }
                        }
                    }
                }
            }
        }

        // Telemetry
        if (Constants.Vision.logTelemetry) {
            telemetryPackets.clear();
            telemetryPackets.add(new TelemetryPacket("PoseCorrectionEnabled", enablePoseCorrection));
            telemetryPackets.add(new TelemetryPacket("Vision Used This Cycle", visionUsed));
            telemetryPackets.add(new TelemetryPacket("Using MT2", usingMT2));
            telemetryPackets.add(new TelemetryPacket("Odo X (FTC)", odometryPose.getX()));
            telemetryPackets.add(new TelemetryPacket("Odo Y (FTC)", odometryPose.getY()));
            telemetryPackets.add(new TelemetryPacket("Odo Heading(rad)", robotHeading));
            telemetryPackets.add(new TelemetryPacket("Turret Angle(deg)", turret.getCurrentTurretAngle()));
            telemetryPackets.add(new TelemetryPacket("Camera Heading(deg)", Math.toDegrees(cameraHeading)));
            telemetryPackets.add(new TelemetryPacket("Vision Valid", results != null && results.isValid()));
            telemetryPackets.add(new TelemetryPacket("Results Null", results == null));
            if (results != null && results.isValid()) {
                Pose3D mt2Pose = results.getBotpose_MT2();
                Pose3D mt1Pose = results.getBotpose();
                telemetryPackets.add(new TelemetryPacket("MT2 Null", mt2Pose == null));
                telemetryPackets.add(new TelemetryPacket("MT1 Null", mt1Pose == null));
                List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
                telemetryPackets.add(new TelemetryPacket("Tags Detected", fiducials != null ? fiducials.size() : 0));
                if (mt2Pose != null) {
                    telemetryPackets.add(new TelemetryPacket("MT2 X (m)", mt2Pose.getPosition().x));
                    telemetryPackets.add(new TelemetryPacket("MT2 Y (m)", mt2Pose.getPosition().y));
                    telemetryPackets.add(new TelemetryPacket("MT2 Heading(deg)", mt2Pose.getOrientation().getYaw()));
                }
                if (mt1Pose != null) {
                    telemetryPackets.add(new TelemetryPacket("MT1 X (m)", mt1Pose.getPosition().x));
                    telemetryPackets.add(new TelemetryPacket("MT1 Y (m)", mt1Pose.getPosition().y));
                    telemetryPackets.add(new TelemetryPacket("MT1 Heading(deg)", mt1Pose.getOrientation().getYaw()));
                }
            }
            if (visionX != null) {
                telemetryPackets.add(new TelemetryPacket("Vision->Robot X (FTC)", visionX));
                telemetryPackets.add(new TelemetryPacket("Vision->Robot Y (FTC)", visionY));
                telemetryPackets.add(new TelemetryPacket("Vision->Robot Heading(rad)", visionHeading));
            }
            telemetryPackets.add(new TelemetryPacket("Drift X", xDriftFilter.getDrift()));
            telemetryPackets.add(new TelemetryPacket("Drift Y", yDriftFilter.getDrift()));
            telemetryPackets.add(new TelemetryPacket("Drift Heading(rad)", headingDriftFilter.getDrift()));
            if (correctedX != null) {
                telemetryPackets.add(new TelemetryPacket("Corrected X (FTC)", correctedX));
                telemetryPackets.add(new TelemetryPacket("Corrected Y (FTC)", correctedY));
                telemetryPackets.add(new TelemetryPacket("Corrected Heading(rad)", correctedHeading));
            }
        }
    }

    /**
     * Convert Limelight botpose to FTC coordinates in inches.
     * Limelight already returns FTC coordinates in meters, so we just convert to inches.
     * No axis swap or origin shift needed since we stay in FTC coordinates.
     *
     * @param botPose Raw Limelight botpose (FTC coords, meters, center origin)
     * @return Pose in FTC coordinates (inches, center origin), or null if invalid
     */
    private Pose getCameraPoseFromLimelight(Pose3D botPose) {
        double rawXMeters = botPose.getPosition().x;
        double rawYMeters = botPose.getPosition().y;

        // Validate: FTC field is ~3.66m x 3.66m, allow margin
        if (rawXMeters < -2.5 || rawXMeters > 2.5 || rawYMeters < -2.5 || rawYMeters > 2.5) {
            return null;
        }

        // Convert meters to inches - stays in FTC coordinates (center origin)
        double ftcXInches = rawXMeters * 39.3701;
        double ftcYInches = rawYMeters * 39.3701;
        double headingRad = Math.toRadians(botPose.getOrientation().getYaw());

        return new Pose(ftcXInches, ftcYInches, headingRad);
    }

    /**
     * Transform from camera pose to robot center pose.
     * Accounts for turret rotation and camera mounting offset.
     * All in FTC coordinates.
     *
     * @param cameraPose Camera pose in FTC coordinates
     * @return Robot center pose in FTC coordinates
     */
    private Pose transformCameraToRobot(Pose cameraPose) {
        double turretAngleRad = Math.toRadians(turret.getCurrentTurretAngle());

        // Robot heading = camera heading - turret angle - mounting offset
        double robotHeading = cameraPose.getHeading() - turretAngleRad
                - Math.toRadians(Constants.Vision.LIMELIGHT_HEADING_OFFSET);

        // Camera offset from robot center (robot-relative, at turret 0°)
        double offsetX = Constants.Vision.LIMELIGHT_X_OFFSET;
        double offsetY = Constants.Vision.LIMELIGHT_Y_OFFSET;

        // Rotate offset by camera's field heading (robot heading + turret angle)
        double cameraFieldHeading = robotHeading + turretAngleRad;
        double rotatedOffsetX = offsetX * Math.cos(cameraFieldHeading) - offsetY * Math.sin(cameraFieldHeading);
        double rotatedOffsetY = offsetX * Math.sin(cameraFieldHeading) + offsetY * Math.cos(cameraFieldHeading);

        // Robot center = camera position - rotated offset
        return new Pose(
                cameraPose.getX() - rotatedOffsetX,
                cameraPose.getY() - rotatedOffsetY,
                robotHeading
        );
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

    public double getRangeFromTag() {
        if (results != null && results.isValid()) {
            Pose3D botPose = results.getBotpose_MT2();
            if (botPose == null) botPose = results.getBotpose();
            if (botPose != null) {
                return Math.sqrt(
                        Math.pow(botPose.getPosition().x, 2) +
                        Math.pow(botPose.getPosition().y, 2)
                );
            }
        }
        return 0.0;
    }

    public void readMotif() {
        if (results == null || !results.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();

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

    public void resetDriftFilters() {
        xDriftFilter = new DriftKalmanFilter(Constants.Vision.X_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_X_STD, Constants.Vision.LOOP_TIME);
        yDriftFilter = new DriftKalmanFilter(Constants.Vision.Y_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_Y_STD, Constants.Vision.LOOP_TIME);
        headingDriftFilter = new DriftKalmanFilter(Constants.Vision.HEADING_DRIFT_SIGMA, Constants.Vision.LIMELIGHT_HEADING_STD, Constants.Vision.LOOP_TIME);
    }

    public void setEnablePoseCorrection(boolean enable) {
        this.enablePoseCorrection = enable;
    }

    /**
     * Set the robot's initial pose using vision (MT2 preferred, MT1 fallback).
     * All math in FTC coordinates, converts to Pedro when setting follower pose.
     * @return true if pose was successfully set
     */
    public boolean setInitialPoseFromVision() {
        read();

        if (results != null && results.isValid()) {
            Pose3D botPose = results.getBotpose_MT2();
            if (botPose == null ||
                Math.abs(botPose.getPosition().x) > 2.5 ||
                Math.abs(botPose.getPosition().y) > 2.5) {
                botPose = results.getBotpose();
            }
            if (botPose != null) {
                Pose cameraPose = getCameraPoseFromLimelight(botPose);
                if (cameraPose != null) {
                    Pose robotPose = transformCameraToRobot(cameraPose);

                    // FTC bounds check: center origin, ±72 inches with margin
                    if (robotPose.getX() >= -82 && robotPose.getX() <= 82 &&
                        robotPose.getY() >= -82 && robotPose.getY() <= 82) {

                        // Convert FTC -> Pedro for the follower
                        setFollowerPoseFromFTC(robotPose);
                        resetDriftFilters();
                        return true;
                    }
                }
            }
        }
        return false;
    }

    public com.seattlesolvers.solverslib.command.InstantCommand setInitialPose() {
        return new com.seattlesolvers.solverslib.command.InstantCommand(this::setInitialPoseFromVision);
    }

    /**
     * Get raw vision pose in FTC coordinates for tuning/telemetry.
     * @return [x, y, heading] in FTC coordinates (inches, center origin), or null
     */
    public double[] getVisionPose() {
        if (results != null && results.isValid()) {
            Pose3D botPose = results.getBotpose_MT2();
            if (botPose == null) botPose = results.getBotpose();
            if (botPose != null) {
                Pose cameraPose = getCameraPoseFromLimelight(botPose);
                if (cameraPose != null) {
                    return new double[]{cameraPose.getX(), cameraPose.getY(), cameraPose.getHeading()};
                }
            }
        }
        return null;
    }

    /**
     * Get MT1 pose in FTC coordinates.
     * @return [x, y, heading] in FTC coordinates, or null
     */
    public double[] getMT1Pose() {
        return getVisionPose();
    }

    /**
     * 1D Kalman Filter that estimates only drift (not position).
     */
    private static class DriftKalmanFilter {
        private double drift;
        private double P;
        private final double Q;
        private final double R;
        private final double maxDrift;

        public DriftKalmanFilter(double driftSigma, double visionStd, double dt) {
            this(driftSigma, visionStd, dt, 36.0);
        }

        public DriftKalmanFilter(double driftSigma, double visionStd, double dt, double maxDrift) {
            this.drift = 0.0;
            this.P = 0.5;
            this.Q = driftSigma * driftSigma * dt;
            this.R = visionStd * visionStd;
            this.maxDrift = maxDrift;
        }

        public void predict() {
            P = P + Q;
        }

        public void update(double visionMeasurement, double odometryReading) {
            double measuredDrift = odometryReading - visionMeasurement;
            double y = measuredDrift - drift;
            double s = P + R;
            double k = P / s;
            drift = drift + k * y;
            drift = Math.max(-maxDrift, Math.min(maxDrift, drift));
            P = (1 - k) * P;
        }

        public void updateAngle(double visionMeasurement, double odometryReading) {
            double measuredDrift = normalizeAngleStatic(odometryReading - visionMeasurement);
            double y = normalizeAngleStatic(measuredDrift - drift);
            double s = P + R;
            double k = P / s;
            drift = drift + k * y;
            drift = normalizeAngleStatic(drift);
            P = (1 - k) * P;
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

