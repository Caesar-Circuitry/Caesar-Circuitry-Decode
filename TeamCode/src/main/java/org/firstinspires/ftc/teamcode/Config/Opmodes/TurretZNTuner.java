package org.firstinspires.ftc.teamcode.Config.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.ThroughBoreEncoder;

import java.util.ArrayList;
import java.util.List;

/**
 * Ziegler-Nichols Turret PID Tuner using @Configurable
 *
 * Adjust values through the Panels dashboard:
 * 1. Set kI=0, kD=0
 * 2. Increase kP until the system oscillates with constant amplitude
 * 3. Set recordOscillations = true
 * 4. Set calculateTu = true to measure period
 * 5. Set applyZN = true to apply calculated PID values
 */
@TeleOp(name = "Turret ZN Tuner", group = "Tuning")
@Configurable
public class TurretZNTuner extends OpMode {

    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    // Hardware
    private CRServo servo;
    private CRServo servo2;
    private ThroughBoreEncoder turretEncoder;

    // PID values - adjustable via dashboard
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kfLeft = 0.04;
    public static double kfRight = -0.05;

    // Target angle - adjustable via dashboard
    public static double targetAngle = 0;

    // Control flags - adjustable via dashboard
    public static boolean recordOscillations = false;
    public static boolean applyZN = false;
    public static boolean resetIntegral = false;
    public static boolean clearRecording = false;
    public static int peaksNeeded = 4; // Stop recording after this many peaks (reduces oscillation time)

    // Turret state
    private double currentAngle = 0;
    private double error = 0;
    private double lastError = 0;
    private double integral = 0;
    private double power = 0;

    // Gear ratio
    private static final double GEAR_RATIO = Constants.Turret.gearRatio;

    // Oscillation recording
    private final List<Double> oscillationPeaks = new ArrayList<>();
    private final List<Double> peakTimes = new ArrayList<>();
    private final ElapsedTime recordTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastAngle = 0;
    private boolean wasIncreasing = false;

    // ZN results - readable on dashboard
    public static double ultimatePeriodTu = 0;
    public static double ultimateGainKu = 0;
    public static double znCalculatedKp = 0;
    public static double znCalculatedKi = 0;
    public static double znCalculatedKd = 0;

    // Status
    public static int peaksDetected = 0;
    public static double lastPeriod = 0;

    @Override
    public void init() {
        // Initialize hardware
        servo = hardwareMap.get(CRServo.class, Constants.Turret.servoName);
        servo2 = hardwareMap.get(CRServo.class, Constants.Turret.servoName2);
        DcMotorEx encoderMotor = hardwareMap.get(DcMotorEx.class, Constants.Turret.encoderMotorName);
        turretEncoder = new ThroughBoreEncoder(encoderMotor, Constants.Turret.gearRatio, Constants.Turret.TICKS_PER_REV, true);

        // Set initial PID values from Constants (only if not already set by dashboard)
        if (kP == 0.01) {
            kP = Constants.Turret.kP_large;
            kI = Constants.Turret.kI_large;
            kD = Constants.Turret.kD_large;
            kfLeft = Constants.Turret.kF_left;
            kfRight = Constants.Turret.kF_right;
        }

        panelsTelemetry.getTelemetry().addData("Status", "Turret ZN Tuner Ready");
        panelsTelemetry.getTelemetry().addData("Instructions", "Use Panels dashboard to adjust values");
        panelsTelemetry.getTelemetry().update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        recordTimer.reset();
        turretEncoder.update();
        currentAngle = turretEncoder.getUnwrappedEncoderAngle() / GEAR_RATIO;
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Update encoder
        turretEncoder.update();
        double unwrappedServoAngle = turretEncoder.getUnwrappedEncoderAngle();
        currentAngle = unwrappedServoAngle / GEAR_RATIO;

        // Handle dashboard control flags
        handleControlFlags();

        // Calculate PID
        error = targetAngle - currentAngle;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Anti-windup
        integral = clamp(integral, -100, 100);

        // Calculate power
        double pTerm = kP * error;
        double iTerm = kI * integral;
        double dTerm = kD * derivative;

        // Feedforward
        double feedforward = 0;
        if (error > 0) {
            feedforward = kfLeft;
        } else if (error < 0) {
            feedforward = kfRight;
        }

        power = pTerm + iTerm + dTerm + feedforward;
        power = clamp(power, -1.0, 1.0);

        // Apply power
        servo.setPower(power);
        servo2.setPower(power);

        // Record oscillations if enabled
        if (recordOscillations) {
            recordOscillationPeaks();
        }

        // Display telemetry
        displayTelemetry(pTerm, iTerm, dTerm, feedforward, dt);
    }

    private void handleControlFlags() {
        // Reset integral if requested
        if (resetIntegral) {
            integral = 0;
            resetIntegral = false;
        }

        // Clear recording data if requested
        if (clearRecording) {
            oscillationPeaks.clear();
            peakTimes.clear();
            peaksDetected = 0;
            lastPeriod = 0;
            clearRecording = false;
        }

        // Apply ZN values if requested
        if (applyZN) {
            applyZNValues();
            applyZN = false;
        }
    }

    private void recordOscillationPeaks() {
        // Detect peaks (local maxima) in the oscillation
        boolean isIncreasing = currentAngle > lastAngle;

        // Peak detected when we were increasing and now decreasing
        if (wasIncreasing && !isIncreasing) {
            oscillationPeaks.add(currentAngle);
            peakTimes.add(recordTimer.seconds());
            peaksDetected = oscillationPeaks.size();

            // Update last period
            if (peakTimes.size() >= 2) {
                lastPeriod = peakTimes.get(peakTimes.size() - 1) - peakTimes.get(peakTimes.size() - 2);

                // Real-time Tu calculation - update after every new peak
                calculateTuRealTime();
            }

            // Auto-stop after enough peaks to minimize oscillation time
            if (peaksDetected >= peaksNeeded) {
                recordOscillations = false;
                // Save current kP as Ku since we found sustained oscillation
                ultimateGainKu = kP;
            }
        }

        wasIncreasing = isIncreasing;
        lastAngle = currentAngle;
    }

    private void calculateTuRealTime() {
        if (peakTimes.size() < 2) {
            return;
        }

        // Calculate average period between peaks
        double totalPeriod = 0;
        int periodCount = 0;

        for (int i = 1; i < peakTimes.size(); i++) {
            double period = peakTimes.get(i) - peakTimes.get(i - 1);
            totalPeriod += period;
            periodCount++;
        }

        if (periodCount > 0) {
            ultimatePeriodTu = totalPeriod / periodCount;
            ultimateGainKu = kP; // Current kP is the ultimate gain

            // Auto-calculate ZN values in real-time
            calculateZNValues();
        }
    }

    private void calculateZNValues() {
        if (ultimateGainKu <= 0 || ultimatePeriodTu <= 0) {
            return;
        }

        // Ziegler-Nichols Classic PID formulas
        // Kp = 0.6 * Ku
        // Ti = 0.5 * Tu (integral time)
        // Td = 0.125 * Tu (derivative time)
        // Ki = Kp / Ti
        // Kd = Kp * Td

        znCalculatedKp = 0.6 * ultimateGainKu;
        double ti = 0.5 * ultimatePeriodTu;
        double td = 0.125 * ultimatePeriodTu;
        znCalculatedKi = znCalculatedKp / ti;
        znCalculatedKd = znCalculatedKp * td;
    }

    private void applyZNValues() {
        if (znCalculatedKp > 0) {
            kP = znCalculatedKp;
            kI = znCalculatedKi;
            kD = znCalculatedKd;
            integral = 0; // Reset integral when applying new values
        }
    }

    private void displayTelemetry(double pTerm, double iTerm, double dTerm, double ff, double dt) {
        panelsTelemetry.getTelemetry().addData("=== TURRET ZN TUNER ===", "");

        // Current PID values
        panelsTelemetry.getTelemetry().addData("kP", kP);
        panelsTelemetry.getTelemetry().addData("kI", kI);
        panelsTelemetry.getTelemetry().addData("kD", kD);

        // Turret state
        panelsTelemetry.getTelemetry().addData("Current Angle", currentAngle);
        panelsTelemetry.getTelemetry().addData("Target Angle", targetAngle);
        panelsTelemetry.getTelemetry().addData("Error", error);
        panelsTelemetry.getTelemetry().addData("Power", power);

        // PID terms
        panelsTelemetry.getTelemetry().addData("P Term", pTerm);
        panelsTelemetry.getTelemetry().addData("I Term", iTerm);
        panelsTelemetry.getTelemetry().addData("D Term", dTerm);
        panelsTelemetry.getTelemetry().addData("Feedforward", ff);

        // Oscillation recording
        panelsTelemetry.getTelemetry().addData("Recording", recordOscillations ? "YES" : "NO");
        panelsTelemetry.getTelemetry().addData("Peaks Detected", peaksDetected);
        panelsTelemetry.getTelemetry().addData("Last Period (sec)", lastPeriod);

        // ZN results
        panelsTelemetry.getTelemetry().addData("Ku (Ultimate Gain)", ultimateGainKu);
        panelsTelemetry.getTelemetry().addData("Tu (Ultimate Period)", ultimatePeriodTu);
        panelsTelemetry.getTelemetry().addData("ZN kP", znCalculatedKp);
        panelsTelemetry.getTelemetry().addData("ZN kI", znCalculatedKi);
        panelsTelemetry.getTelemetry().addData("ZN kD", znCalculatedKd);

        panelsTelemetry.getTelemetry().addData("Loop (ms)", dt * 1000);

        panelsTelemetry.getTelemetry().update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        servo.setPower(0);
        servo2.setPower(0);
    }
}

