package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
@TeleOp
public class DualPIDContinuousServoTesterPinpoint extends LinearOpMode {
  private CRServo servo;
  private CRServo servo2;
  private AnalogInput servoEncoder; // encoder based on servo pos
  private GoBildaPinpointDriver pinpoint;
  private PIDFController angleController;

  private int encoderWraps = 0;
  private double lastRawEncoderAngle = 0;
  private static final double WRAP_THRESHOLD = 270.0;

  // PID gain sets
  public static boolean useLargePID = true; // Toggle: true = large PID, false = small PID

  public static double kP_large = 0.008;
  public static double kI_large = 0;
  public static double kD_large = 0;

  // Small PID gains (precise, smooth response)
  public static double kP_small = 0.02;
  public static double kI_small = 0;
  public static double kD_small = 0;

  // Active PID gains (will be set based on useLargePID)
  public static double kP = 0.01;
  public static double kI = 0;
  public static double kD = 0;

  // Feedforward gains
  public static double kF_left = 0.07; // Feedforward when turning left (positive error)
  public static double kF_right = -0.1; // Feedforward when turning right (negative error)


  // Hardware names
  public static String servoName = "servo1";
  public static String servoName2 = "servo2";
  public static String servoEncoderName = "rightServo";

  // Control parameters
  public static double targetAngle = 0; // field-relative angle the turret should face
  public static double gearRatio =
      2; // servo rotations per turret rotation (2:1 = servo rotates 2x)
  private JoinedTelemetry Telemetry;

  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(CRServo.class, servoName);
    servo2 = hardwareMap.get(CRServo.class, servoName2);
    servoEncoder = hardwareMap.get(AnalogInput.class, servoEncoderName);
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    // Initialize PID controller with configurable values
    angleController = new PIDFController(kP, kI, kD, 0);


    // Configure the sensor
    configurePinpoint();

    // Set the location of the robot - this should be the place you are starting the robot from
    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    waitForStart();

    while (opModeIsActive()) {
      // Update pinpoint position
      pinpoint.update();
      double heading = pinpoint.getHeading(AngleUnit.DEGREES);

      // Select PID gains based on toggle
      if (useLargePID) {
        kP = kP_large;
        kI = kI_large;
        kD = kD_large;
      } else {
        kP = kP_small;
        kI = kI_small;
        kD = kD_small;
      }

      // Calculate desired turret angle (robot-relative) to point to targetAngle (field-relative)
      double desiredTurretAngleRaw = targetAngle - heading;
      double wrappedDesiredTurretAngle = wrap180(desiredTurretAngleRaw);

      // Find the closest valid turret angle within the 270-degree range (-135 to +135)
      double desiredTurretAngle = getClosestAngleInRange(wrappedDesiredTurretAngle, -135.0, 135.0);

      // Convert desired turret angle to desired servo angle using gear ratio
      double desiredServoAngle = desiredTurretAngle * gearRatio;

      // Read current servo angle from absolute encoder (in degrees)
      double currentServoAngle = getUnwrappedServoAngle(servoEncoder);

      // Convert current servo angle to equivalent turret angle for telemetry
      double currentTurretAngle = currentServoAngle / gearRatio;


      // Update PID controller coefficients in case they changed via dashboard
      angleController.setPIDF(kP, kI, kD, 0);

      // Calculate servo power using PID control
      double servoError = desiredServoAngle - currentServoAngle;
      double wrappedServoError = wrap180(servoError);

      // Calculate the target position that's closest to current via wrapping
      double adjustedTarget = currentServoAngle + wrappedServoError;

      // Use PID controller with current position and adjusted target
      angleController.setSetPoint(adjustedTarget);
      double servoPower = angleController.calculate(currentServoAngle);

      // Add direction-specific feedforward term to overcome friction/deadband
      // Both directions add positive feedforward in the direction of motion
      if (wrappedServoError > 0) {
        servoPower += kF_left;  // Moving left (positive power)
      } else if (wrappedServoError < 0) {
        servoPower += kF_right; // Moving right (negative power) - changed from -= to +=
      }

      // Clamp servo power to valid range for CRServo
      servoPower = clamp(servoPower, -1.0, 1.0);

      // Set power to both servos
      servo.setPower(servoPower);
      servo2.setPower(servoPower);

      // Telemetry for debugging
      Telemetry.addData("Target Field Angle", "%.2f deg", targetAngle);
      Telemetry.addData("Robot Heading", "%.2f deg", heading);
      Telemetry.addData("Desired Turret Angle (robot-relative)", "%.2f deg", desiredTurretAngle);
      Telemetry.addData("Current Turret Angle (robot-relative)", "%.2f deg", currentTurretAngle);
      Telemetry.addData("---", "---");
      Telemetry.addData("PID Mode", useLargePID ? "LARGE (Aggressive)" : "SMALL (Precise)");
      Telemetry.addData("Active kP / kI / kD", "%.4f / %.4f / %.4f", kP, kI, kD);
      Telemetry.addData("", "");
      Telemetry.addData("Large PID", "P=%.4f I=%.4f D=%.4f", kP_large, kI_large, kD_large);
      Telemetry.addData("Small PID", "P=%.4f I=%.4f D=%.4f", kP_small, kI_small, kD_small);
      Telemetry.addData("---", "---");
      Telemetry.addData("Raw Encoder Angle", "%.2f deg", getCurrentPosition(servoEncoder));
      Telemetry.addData("Encoder Wraps", encoderWraps);
      Telemetry.addData("Current Servo Angle (unwrapped)", "%.2f deg", currentServoAngle);
      Telemetry.addData("Desired Servo Angle", "%.2f deg", desiredServoAngle);
      Telemetry.addData("Servo Error", "%.2f deg", wrappedServoError);
      Telemetry.addData("---", "---");
      Telemetry.addData("kF (Left / Right)", "%.4f / %.4f", kF_left, kF_right);
      Telemetry.addData("Servo Encoder Voltage", "%.2f V", servoEncoder.getVoltage());
      Telemetry.addData("Servo Power", "%.3f", servoPower);
      Telemetry.addData("Gear Ratio", "%.1f:1", gearRatio);
      Telemetry.update();
    }
  }

  public void configurePinpoint() {
    /*
     *  Set the odometry pod positions relative to the point that you want the position to be measured from.
     *
     *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
     *  Left of the center is a positive number, right of center is a negative number.
     *
     *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
     *  Forward of center is a positive number, backwards is a negative number.
     */
    pinpoint.setOffsets(
        -84.0, -168.0, DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1

    /*
     * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
     * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
     * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
     * number of ticks per unit of your odometry pod.  For example:
     *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
     */
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    /*
     * Set the direction that each of the two odometry pods count. The X (forward) pod should
     * increase when you move the robot forward. And the Y (strafe) pod should increase when
     * you move the robot to the left.
     */
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /*
     * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
     * The IMU will automatically calibrate when first powered on, but recalibrating before running
     * the robot is a good idea to ensure that the calibration is "good".
     * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
     * This is recommended before you run your autonomous, as a bad initial calibration can cause
     * an incorrect starting value for x, y, and heading.
     */
    pinpoint.resetPosAndIMU();
  }

  private double wrap180(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
  }

  private double getClosestAngleInRange(double targetAngle, double minAngle, double maxAngle) {
    // If target is within range, use it directly
    if (targetAngle >= minAngle && targetAngle <= maxAngle) {
      return targetAngle;
    }

    // Target is out of range, find the closest boundary
    // Consider wrap-around: the target might be closer via the other side of the circle

    // Calculate distance to each boundary
    double distToMin = Math.abs(wrap180(targetAngle - minAngle));
    double distToMax = Math.abs(wrap180(targetAngle - maxAngle));

    // Also consider wrapping to the opposite side
    double wrappedTarget = targetAngle > 0 ? targetAngle - 360.0 : targetAngle + 360.0;
    double distToMinWrapped = Math.abs(wrap180(wrappedTarget - minAngle));
    double distToMaxWrapped = Math.abs(wrap180(wrappedTarget - maxAngle));

    // Find minimum distance
    double minDist =
        Math.min(Math.min(distToMin, distToMax), Math.min(distToMinWrapped, distToMaxWrapped));

    // Return the boundary with minimum distance
    if (minDist == distToMin || minDist == distToMinWrapped) {
      return minAngle;
    } else {
      return maxAngle;
    }
  }

  private double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  private double getUnwrappedServoAngle(AnalogInput analogInput) {
    double rawAngle = getCurrentPosition(analogInput);

    // Detect wrap-around
    double angleDelta = rawAngle - lastRawEncoderAngle;

    // If we jumped from near +180 to near -180 (positive wrap)
    if (angleDelta < -WRAP_THRESHOLD) {
      encoderWraps++;
    }
    // If we jumped from near -180 to near +180 (negative wrap)
    else if (angleDelta > WRAP_THRESHOLD) {
      encoderWraps--;
    }

    lastRawEncoderAngle = rawAngle;

    // Calculate unwrapped angle
    return rawAngle + (encoderWraps * 360.0);
  }

  private double getCurrentPosition(AnalogInput analogInput) {
    return ((analogInput.getVoltage() / 3.3) * 360) - 180;
  }
}
