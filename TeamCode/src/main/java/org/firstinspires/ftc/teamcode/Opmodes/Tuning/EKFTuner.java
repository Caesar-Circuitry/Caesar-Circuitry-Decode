package org.firstinspires.ftc.teamcode.Opmodes.Tuning;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "EKF Tuner", group = "Tuning")
public class EKFTuner extends OpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private Pose startingPose = new Pose();

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime sampleTimer = new ElapsedTime();

    // Data collection arrays
    private List<Double> xSamples = new ArrayList<>();
    private List<Double> ySamples = new ArrayList<>();
    private List<Double> headingSamples = new ArrayList<>();
    private List<Double> visionXSamples = new ArrayList<>();
    private List<Double> visionYSamples = new ArrayList<>();
    private List<Double> visionHeadingSamples = new ArrayList<>();

    private double xSum = 0, ySum = 0, headingSum = 0;
    private double visionXSum = 0, visionYSum = 0, visionHeadingSum = 0;
    private int readingsInSecond = 0;
    private int visionReadingsInSecond = 0;

    private static final double SAMPLE_INTERVAL = 1.0; // 1 second intervals
    private static final int MAX_SAMPLES = 100; // Collect 100 samples (100 seconds)

    private enum TuningState {
        WAITING_TO_START,
        COLLECTING_DRIFT_DATA,
        COLLECTING_VISION_DATA,
        CALCULATING_RESULTS,
        COMPLETE
    }

    private TuningState state = TuningState.WAITING_TO_START;

    // Calculated results
    private double xDriftSigma = 0;
    private double yDriftSigma = 0;
    private double headingDriftSigma = 0;
    private double visionXStd = 0;
    private double visionYStd = 0;
    private double visionHeadingStd = 0;

    @Override
    public void init() {
        robot = new robot(hardwareMap);
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        telemetry.addLine("EKF Tuner");
        telemetry.addLine("This will measure Q and R matrices for the Kalman filter");
        telemetry.addLine();
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Place robot stationary on field");
        telemetry.addLine("2. Ensure AprilTags are visible");
        telemetry.addLine("3. Press START (flywheel will start automatically)");
        telemetry.addLine();
        telemetry.addLine("Test duration: ~3.5 minutes (200 seconds)");
        telemetry.addLine("DO NOT MOVE ROBOT during test!");
        telemetry.update();
        telemetry.addLine("Test duration: ~3.5 minutes (200 seconds)");
        telemetry.addLine("DO NOT MOVE ROBOT during test!");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.getHardware().getFollower().setStartingPose(startingPose);
        robot.getHardware().getFollower().getPose(); // Initialize pose
        timer.reset();
        sampleTimer.reset();
        state = TuningState.COLLECTING_DRIFT_DATA;
        telemetry.clear();

        // Start the flywheel/launcher for vibration simulation
        robot.getHardware().getLauncher().setFlywheelTargetVelocity(Constants.Launcher.FarVelocity);
    }

    @Override
    public void loop() {
        robot.read();
        robot.loop();
        robot.write();

        switch (state) {
            case COLLECTING_DRIFT_DATA:
                collectDriftData();
                break;
            case COLLECTING_VISION_DATA:
                collectVisionData();
                break;
            case CALCULATING_RESULTS:
                calculateResults();
                break;
            case COMPLETE:
                displayResults();
                break;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the flywheel/launcher when tuning is complete or stopped
        robot.getHardware().getLauncher().setStopPower(true);
    }

    private void collectDriftData() {
        // Get current odometry pose
        double x = robot.getHardware().getFollower().getPose().getX();
        double y = robot.getHardware().getFollower().getPose().getY();
        double heading = robot.getHardware().getFollower().getPose().getHeading();

        // Accumulate readings
        xSum += x;
        ySum += y;
        headingSum += heading;
        readingsInSecond++;

        // Every second, save the average
        if (sampleTimer.seconds() >= SAMPLE_INTERVAL) {
            double xAvg = xSum / readingsInSecond;
            double yAvg = ySum / readingsInSecond;
            double headingAvg = headingSum / readingsInSecond;

            xSamples.add(xAvg);
            ySamples.add(yAvg);
            headingSamples.add(headingAvg);

            // Reset for next second
            xSum = 0;
            ySum = 0;
            headingSum = 0;
            readingsInSecond = 0;
            sampleTimer.reset();

            telemetry.addLine("=== DRIFT DATA COLLECTION ===");
            telemetry.addData("Phase", "Measuring Pinpoint drift (Q matrix)");
            telemetry.addData("Samples collected", xSamples.size() + " / " + MAX_SAMPLES);
            telemetry.addData("Time remaining", (MAX_SAMPLES - xSamples.size()) + " seconds");
            telemetry.addLine();
            telemetry.addLine("Keep robot STATIONARY with flywheel RUNNING");
            telemetry.addData("Current X drift", xAvg);
            telemetry.addData("Current Y drift", yAvg);
            telemetry.addData("Current Heading drift", Math.toDegrees(headingAvg));

            // Move to next phase after MAX_SAMPLES
            if (xSamples.size() >= MAX_SAMPLES) {
                state = TuningState.COLLECTING_VISION_DATA;
                sampleTimer.reset();
                telemetry.clear();
                telemetry.addLine("Drift data collection complete!");
                telemetry.addLine("Starting vision data collection...");
                telemetry.addLine("Make sure AprilTags are visible!");
            }
        }
    }

    private void collectVisionData() {
        // Get vision pose measurements
        double[] visionPose = robot.getHardware().getVision().getVisionPose();

        if (visionPose != null) {
            // Accumulate vision readings
            visionXSum += visionPose[0];
            visionYSum += visionPose[1];
            visionHeadingSum += visionPose[2];
            visionReadingsInSecond++;
        }

        // Every second, save the average
        if (sampleTimer.seconds() >= SAMPLE_INTERVAL) {
            if (visionReadingsInSecond > 0) {
                double visionXAvg = visionXSum / visionReadingsInSecond;
                double visionYAvg = visionYSum / visionReadingsInSecond;
                double visionHeadingAvg = visionHeadingSum / visionReadingsInSecond;

                visionXSamples.add(visionXAvg);
                visionYSamples.add(visionYAvg);
                visionHeadingSamples.add(visionHeadingAvg);
            }

            // Reset for next second
            visionXSum = 0;
            visionYSum = 0;
            visionHeadingSum = 0;
            visionReadingsInSecond = 0;
            sampleTimer.reset();

            telemetry.addLine("=== VISION DATA COLLECTION ===");
            telemetry.addData("Phase", "Measuring Limelight noise (R matrix)");
            telemetry.addData("Samples collected", visionXSamples.size() + " / " + MAX_SAMPLES);
            telemetry.addData("Time remaining", (MAX_SAMPLES - visionXSamples.size()) + " seconds");
            telemetry.addLine();
            telemetry.addLine("Keep robot STATIONARY with AprilTags VISIBLE");
            telemetry.addData("Readings this second", visionReadingsInSecond);

            // Move to calculation phase after MAX_SAMPLES
            if (visionXSamples.size() >= MAX_SAMPLES) {
                state = TuningState.CALCULATING_RESULTS;
            }
        }
    }

    private void calculateResults() {
        telemetry.addLine("Calculating results...");

        // Calculate drift sigma (Q matrix components)
        xDriftSigma = calculateDriftSigma(xSamples);
        yDriftSigma = calculateDriftSigma(ySamples);
        headingDriftSigma = calculateDriftSigma(headingSamples);

        // Calculate vision standard deviation (R matrix components)
        visionXStd = calculateStandardDeviation(visionXSamples);
        visionYStd = calculateStandardDeviation(visionYSamples);
        visionHeadingStd = calculateStandardDeviation(visionHeadingSamples);

        state = TuningState.COMPLETE;
    }

    private double calculateDriftSigma(List<Double> samples) {
        if (samples.size() < 2) return 0;

        // Method 1: First and last measurement divided by sqrt(time)
        double method1 = Math.abs(samples.get(samples.size() - 1) - samples.get(0)) /
                        Math.sqrt(samples.size() * SAMPLE_INTERVAL);

        // Method 2: Average of all 1-second differences
        double sumDiff = 0;
        for (int i = 1; i < samples.size(); i++) {
            sumDiff += Math.abs(samples.get(i) - samples.get(i - 1));
        }
        double method2 = sumDiff / (samples.size() - 1) / Math.sqrt(SAMPLE_INTERVAL);

        // Method 3: Average of 4-second differences divided by 2
        double sumDiff4 = 0;
        int count = 0;
        for (int i = 4; i < samples.size(); i++) {
            sumDiff4 += Math.abs(samples.get(i) - samples.get(i - 4));
            count++;
        }
        double method3 = (count > 0) ? (sumDiff4 / count) / Math.sqrt(4 * SAMPLE_INTERVAL) / 2 : method2;

        // Return average of all methods
        return (method1 + method2 + method3) / 3.0;
    }

    private double calculateStandardDeviation(List<Double> samples) {
        if (samples.isEmpty()) return 0;

        // Calculate mean
        double mean = 0;
        for (double sample : samples) {
            mean += sample;
        }
        mean /= samples.size();

        // Calculate variance
        double variance = 0;
        for (double sample : samples) {
            variance += Math.pow(sample - mean, 2);
        }
        variance /= samples.size();

        // Return standard deviation
        return Math.sqrt(variance);
    }

    private void displayResults() {
        telemetry.addLine("=== EKF TUNING RESULTS ===");
        telemetry.addLine();
        telemetry.addLine("--- DRIFT SIGMA (for Q matrix) ---");
        telemetry.addData("X_DRIFT_SIGMA", "%.6f inches/sqrt(sec)", xDriftSigma);
        telemetry.addData("Y_DRIFT_SIGMA", "%.6f inches/sqrt(sec)", yDriftSigma);
        telemetry.addData("HEADING_DRIFT_SIGMA", "%.6f rad/sqrt(sec)", headingDriftSigma);
        telemetry.addLine();
        telemetry.addLine("--- VISION STD (for R matrix) ---");
        telemetry.addData("LIMELIGHT_X_STD", "%.6f inches", visionXStd);
        telemetry.addData("LIMELIGHT_Y_STD", "%.6f inches", visionYStd);
        telemetry.addData("LIMELIGHT_HEADING_STD", "%.6f rad", visionHeadingStd);
        telemetry.addLine();
        telemetry.addLine("Copy these values to Vision.java:");
        telemetry.addLine("private static final double X_DRIFT_SIGMA = " + String.format("%.6f", xDriftSigma) + ";");
        telemetry.addLine("private static final double Y_DRIFT_SIGMA = " + String.format("%.6f", yDriftSigma) + ";");
        telemetry.addLine("private static final double HEADING_DRIFT_SIGMA = " + String.format("%.6f", headingDriftSigma) + ";");
        telemetry.addLine("private static final double LIMELIGHT_X_STD = " + String.format("%.6f", visionXStd) + ";");
        telemetry.addLine("private static final double LIMELIGHT_Y_STD = " + String.format("%.6f", visionYStd) + ";");
        telemetry.addLine("private static final double LIMELIGHT_HEADING_STD = " + String.format("%.6f", visionHeadingStd) + ";");
    }
}
