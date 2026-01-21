package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Turret KF Auto Tuner")
public class TurretKfAutoTuner extends LinearOpMode {
  private CRServo servo;
  private CRServo servo2;
  private AnalogInput servoEncoder;

  private int encoderWraps = 0;
  private double lastRawEncoderAngle = 0;
  private static final double WRAP_THRESHOLD = 270.0;

  // Hardware names
  public static String servoName = "servo1";
  public static String servoName2 = "servo2";
  public static String servoEncoderName = "rightServo";
  public static double gearRatio = 2;

  // Tuning parameters
  public static double testAngleRange = 45.0; // How far to move for each test (degrees)
  public static double kfTestStart = 0.0; // Starting kF value (kStatic)
  public static double kfTestEnd = 0.2; // Ending kF value
  public static double kfTestStep = 0.01; // Increment per test
  public static double testDurationSeconds = 2.0; // How long each test runs
  public static double movementThreshold = 0.5; // Minimum movement to detect (degrees)
  public static double basePower = 0.0; // Should be 0 - kF is the static friction value

  private JoinedTelemetry Telemetry;
  private ElapsedTime testTimer;

  // Tuning state
  private enum TuningState {
    PROMPT_LEFT,
    TESTING_LEFT,
    PROMPT_RIGHT,
    TESTING_RIGHT,
    COMPLETE
  }

  private TuningState state = TuningState.PROMPT_LEFT;
  private double currentKfTest = 0;
  private double previousKfTest = 0; // Track previous kF value
  private double bestKfLeft = 0;
  private double bestKfRight = 0;
  private double bestDistanceLeft = 0;
  private double bestDistanceRight = 0;
  private double testStartPosition = 0;
  private double totalDistanceMoved = 0;
  private boolean movementDetected = false; // Track if movement was detected

  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(CRServo.class, servoName);
    servo2 = hardwareMap.get(CRServo.class, servoName2);
    servoEncoder = hardwareMap.get(AnalogInput.class, servoEncoderName);
    Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    testTimer = new ElapsedTime();

    Telemetry.addData("STATUS", "Simple KF Tuner Ready");
    Telemetry.addData("", "");
    Telemetry.addData("How it works:", "");
    Telemetry.addData("", "Tests kF values from %.2f to %.2f", kfTestStart, kfTestEnd);
    Telemetry.addData("", "Measures how far turret moves in %.1fs", testDurationSeconds);
    Telemetry.addData("", "kF = kStatic (minimum power to overcome friction)");
    Telemetry.addData("", "Best kF = moves farthest with just kF");
    Telemetry.addData("", "");
    Telemetry.addData("Instructions", "Press START to begin");
    Telemetry.update();

    waitForStart();

    while (opModeIsActive()) {
      switch (state) {
        case PROMPT_LEFT:
          servo.setPower(0);
          servo2.setPower(0);

          Telemetry.addData("STATUS", "Press A to test LEFT turns");
          Telemetry.addData("", "Will test kF values from %.2f to %.2f", kfTestStart, kfTestEnd);
          Telemetry.addData("", "Turret will move with just kF power (kStatic)");
          Telemetry.update();

          if (gamepad1.a) {
            state = TuningState.TESTING_LEFT;
            currentKfTest = kfTestStart;
            previousKfTest = 0;
            bestDistanceLeft = 0;
            movementDetected = false;
            testTimer.reset();
            testStartPosition = getUnwrappedServoAngle(servoEncoder);
          }
          break;

        case TESTING_LEFT:
          // Apply power = basePower + kF
          double leftPower = basePower + currentKfTest;
          leftPower = clamp(leftPower, -1.0, 1.0);
          servo.setPower(leftPower);
          servo2.setPower(leftPower);

          // Check current position
          double currentPosition = getUnwrappedServoAngle(servoEncoder);
          totalDistanceMoved = Math.abs(currentPosition - testStartPosition);

          // Check if movement detected
          if (totalDistanceMoved > movementThreshold && !movementDetected) {
            movementDetected = true;
            // Use PREVIOUS kF value as the best (the one before movement)
            bestKfLeft = previousKfTest;
            bestDistanceLeft = totalDistanceMoved;
            servo.setPower(0);
            servo2.setPower(0);
            state = TuningState.PROMPT_RIGHT;
            break;
          }

          if (testTimer.seconds() >= testDurationSeconds) {
            // Test complete, no movement detected yet
            // Move to next test
            previousKfTest = currentKfTest;
            currentKfTest += kfTestStep;

            if (currentKfTest > kfTestEnd) {
              // Reached end without finding threshold
              bestKfLeft = kfTestEnd;
              bestDistanceLeft = 0;
              servo.setPower(0);
              servo2.setPower(0);
              state = TuningState.PROMPT_RIGHT;
            } else {
              // Next test
              testTimer.reset();
              testStartPosition = getUnwrappedServoAngle(servoEncoder);
            }
          }

          double progressLeft = (currentKfTest - kfTestStart) / (kfTestEnd - kfTestStart) * 100;
          Telemetry.addData("STATUS", "TESTING LEFT TURNS");
          Telemetry.addData("Current kF (kStatic)", "%.3f", currentKfTest);
          Telemetry.addData("Current Power", "%.3f (just kF)", leftPower);
          Telemetry.addData("Test Time", "%.1f / %.1f s", testTimer.seconds(), testDurationSeconds);
          Telemetry.addData("Distance This Test", "%.1f deg", totalDistanceMoved);
          Telemetry.addData("Movement Detected?", totalDistanceMoved > movementThreshold ? "YES!" : "No");
          Telemetry.addData("Progress", "%.0f%%", progressLeft);
          Telemetry.addData("", "");
          Telemetry.addData("Best kF Left", "%.3f (stopped at %.1f deg)", bestKfLeft, bestDistanceLeft);
          Telemetry.update();
          break;

        case PROMPT_RIGHT:
          servo.setPower(0);
          servo2.setPower(0);

          Telemetry.addData("STATUS", "Press B to test RIGHT turns");
          Telemetry.addData("", "Will test kF values from %.2f to %.2f", kfTestStart, kfTestEnd);
          Telemetry.addData("", "Turret will move with -kF power (kStatic)");
          Telemetry.addData("", "");
          Telemetry.addData("Best kF Left", "%.3f (moved %.1f deg)", bestKfLeft, bestDistanceLeft);
          Telemetry.update();

          if (gamepad1.b) {
            state = TuningState.TESTING_RIGHT;
            currentKfTest = kfTestStart;
            previousKfTest = 0;
            bestDistanceRight = 0;
            movementDetected = false;
            testTimer.reset();
            testStartPosition = getUnwrappedServoAngle(servoEncoder);
          }
          break;

        case TESTING_RIGHT:
          // Apply power = -(basePower + kF)
          double rightPower = -(basePower + currentKfTest);
          rightPower = clamp(rightPower, -1.0, 1.0);
          servo.setPower(rightPower);
          servo2.setPower(rightPower);

          // Check current position
          currentPosition = getUnwrappedServoAngle(servoEncoder);
          totalDistanceMoved = Math.abs(currentPosition - testStartPosition);

          // Check if movement detected
          if (totalDistanceMoved > movementThreshold && !movementDetected) {
            movementDetected = true;
            // Use PREVIOUS kF value as the best (the one before movement)
            bestKfRight = previousKfTest;
            bestDistanceRight = totalDistanceMoved;
            servo.setPower(0);
            servo2.setPower(0);
            state = TuningState.COMPLETE;
            break;
          }

          if (testTimer.seconds() >= testDurationSeconds) {
            // Test complete, no movement detected yet
            // Move to next test
            previousKfTest = currentKfTest;
            currentKfTest += kfTestStep;

            if (currentKfTest > kfTestEnd) {
              // Reached end without finding threshold
              bestKfRight = kfTestEnd;
              bestDistanceRight = 0;
              servo.setPower(0);
              servo2.setPower(0);
              state = TuningState.COMPLETE;
            } else {
              // Next test
              testTimer.reset();
              testStartPosition = getUnwrappedServoAngle(servoEncoder);
            }
          }

          double progressRight = (currentKfTest - kfTestStart) / (kfTestEnd - kfTestStart) * 100;
          Telemetry.addData("STATUS", "TESTING RIGHT TURNS");
          Telemetry.addData("Current kF (kStatic)", "%.3f", currentKfTest);
          Telemetry.addData("Current Power", "%.3f (-kF)", rightPower);
          Telemetry.addData("Test Time", "%.1f / %.1f s", testTimer.seconds(), testDurationSeconds);
          Telemetry.addData("Distance This Test", "%.1f deg", totalDistanceMoved);
          Telemetry.addData("Movement Detected?", totalDistanceMoved > movementThreshold ? "YES!" : "No");
          Telemetry.addData("Progress", "%.0f%%", progressRight);
          Telemetry.addData("", "");
          Telemetry.addData("Best kF Left", "%.3f (moved %.1f deg)", bestKfLeft, bestDistanceLeft);
          Telemetry.addData("Best kF Right", "%.3f (stopped at %.1f deg)", bestKfRight, bestDistanceRight);
          Telemetry.update();
          break;

        case COMPLETE:
          servo.setPower(0);
          servo2.setPower(0);

          Telemetry.addData("STATUS", "✓ TUNING COMPLETE!");
          Telemetry.addData("", "");
          Telemetry.addData("=== RESULTS ===", "");
          Telemetry.addData("Best kF_left", "%.4f", bestKfLeft);
          Telemetry.addData("  Distance", "%.1f degrees", bestDistanceLeft);
          Telemetry.addData("", "");
          Telemetry.addData("Best kF_right", "%.4f", bestKfRight);
          Telemetry.addData("  Distance", "%.1f degrees", bestDistanceRight);
          Telemetry.addData("", "");
          Telemetry.addData("NOTE: kF_right should be NEGATIVE:", "");
          Telemetry.addData("Copy these to your OpMode:", "");
          Telemetry.addData("kF_left", " = %.4f", bestKfLeft);
          Telemetry.addData("kF_right", " = %.4f", -bestKfRight);
          Telemetry.update();
          break;
      }
    }
  }


  private double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  private double getUnwrappedServoAngle(AnalogInput analogInput) {
    double rawAngle = getCurrentPosition(analogInput);
    double angleDelta = rawAngle - lastRawEncoderAngle;

    if (angleDelta < -WRAP_THRESHOLD) {
      encoderWraps++;
    } else if (angleDelta > WRAP_THRESHOLD) {
      encoderWraps--;
    }

    lastRawEncoderAngle = rawAngle;
    return rawAngle + (encoderWraps * 360.0);
  }

  private double getCurrentPosition(AnalogInput analogInput) {
    return ((analogInput.getVoltage() / 3.3) * 360) - 180;
  }
}

